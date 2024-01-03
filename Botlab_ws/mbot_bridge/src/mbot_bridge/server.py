#!/bin/python3

import yaml
import asyncio
import signal
import select
import logging
import threading
import websockets
import time

import lcm
from mbot_bridge.utils import type_utils
from mbot_bridge.utils.json_messages import (
    MBotJSONMessage, MBotJSONResponse, MBotJSONError,
    MBotMessageType, BadMBotRequestError
)


class LCMMessageQueue(object):
    def __init__(self, channel, dtype, queue_size=1):
        self.channel = channel
        self.dtype = dtype
        self.queue_size = queue_size

        self._queue = []
        self._lock = threading.Lock()

    def push(self, msg, decode=True):
        # Decode to LCM type.
        if decode:
            msg = type_utils.decode(msg, self.dtype)

        self._lock.acquire()
        # Add the current message to the back of the queue.
        self._queue.append(msg)
        # Remove old messages if necessary.
        while len(self._queue) > self.queue_size:
            self._queue.pop(0)
        self._lock.release()

    def latest(self):
        latest = None
        self._lock.acquire()
        if len(self._queue) > 0:
            latest = self._queue[-1]
        self._lock.release()
        return latest

    def pop(self):
        first = None
        self._lock.acquire()
        if len(self._queue) > 0:
            first = self._queue.pop(0)
        self._lock.release()
        return first

    def empty(self):
        return len(self._queue) == 0


class MBotBridgeServer(object):
    def __init__(self, lcm_address, subs=[], lcm_timeout=1000, hostfile="/etc/hostname", discard_msgs=-1):
        self._hostname = self._read_hostname(hostfile)
        self._loop = None
        self.discard_msgs = discard_msgs

        # LCM setup.
        self._lcm_timeout = lcm_timeout  # This is how long to timeout in the LCM handle call.
        self._lcm = lcm.LCM(lcm_address)

        self._msg_managers = {}
        self._subs = {}
        for channel in subs:
            ch, lcm_type = channel["channel"], channel["type"]
            self._subs.update({ch: []})

            self._msg_managers.update({ch: LCMMessageQueue(ch, lcm_type)})
            self._lcm.subscribe(ch, self.listener)

        self._running = True
        self._lock = threading.Lock()
        logging.info(f"Discard msgs seconds: {discard_msgs}")
        logging.info(f"Hostname: {self._hostname}")
        logging.info(f"Connecting to LCM on address: {lcm_address}")
        logging.info("Listening on channels:")
        for ch in subs:
            logging.info(f"    {ch['channel']} ({ch['type']})")
        logging.info("MBot Bridge Server running!")

    def stop(self, *args):
        self._lock.acquire()
        self._running = False
        self._lock.release()

    def running(self):
        self._lock.acquire()
        res = self._running
        self._lock.release()
        return res

    def _read_hostname(self, hostfile):
        if not os.path.exists(hostfile):
            logging.warning(f"Host file does not exist, hostname will be empty. Host file: {hostfile}")
            return ""

        # Read the robot's host name.
        with open(hostfile, 'r') as f:
            name = f.read()

        return name.strip()

    def _latest_as_msg(self, ch):
        latest = self._msg_managers[ch].latest()
        latest = type_utils.lcm_type_to_dict(latest)  # Convert to dictionary.
        # Wrap the response data for sending over the websocket.
        res = MBotJSONResponse(latest, ch, self._msg_managers[ch].dtype)
        return res

    def listener(self, channel, data):
        self._msg_managers[channel].push(data, decode=True)

        # If there are subscribers, send them the data.
        if len(self._subs[channel]) > 0:
            res = self._msg_managers[ch].latest()
            for ws_sub in self._subs[channel]:
                try:
                    self._loop.run_until_complete(ws_sub.send(res.encode()))
                except (websockets.exceptions.ConnectionClosedOK,
                        websockets.exceptions.ConnectionClosedError):
                    # If this websocket is closed, remove it.
                    logging.debug(f"Websocket ID {ws_sub.id} - Disconnected and unsubscribed from {channel}")
                    self._subs[channel].remove(ws_sub)

    def handleOnce(self):
        # This is a non-blocking handle, which only calls handle if a message is ready.
        rfds, wfds, efds = select.select([self._lcm.fileno()], [], [], 0)
        if rfds:
            self._lcm.handle()

    def lcm_loop(self):
        self._loop = asyncio.new_event_loop()
        while self.running():
            # This will block for a maximum of _lcm_timeout milliseconds, so it
            # might slow stopping the server, but it's less expensive than using
            # the non-blocking handleOnce.
            self._lcm.handle_timeout(self._lcm_timeout)

    def _subscribe(self, ws, channel):
        self._subs[channel].append(ws)

    def _unsubscribe(self, ws, channel=None):
        self._subs[channel].remove(ws)

    async def process_msg(self, websocket, message):
        try:
            request = MBotJSONMessage(message, from_json=True)
        except BadMBotRequestError as e:
            # If something went wrong parsing this request, send the error message then continue.
            msg = f"Bad MBot request. Ignoring. BadMBotRequestError: {e}"
            logging.warning(f"{websocket.id} - {msg}")
            err = MBotJSONError(msg)
            await websocket.send(err.encode())
            return

        if request.type() == MBotMessageType.REQUEST:
            ch = request.channel()
            if ch == "HOSTNAME":
                # If hostname, return the hostname as a string.
                res = MBotJSONResponse(self._hostname, ch, "")
                await websocket.send(res.encode())
            elif ch not in self._msg_managers:
                # If the channel being requested does not exist, return an error.
                msg = f"Bad MBot request. No channel: {ch}"
                logging.warning(f"{websocket.id} - {msg}")
                err = MBotJSONError(msg)
                await websocket.send(err.encode())
            elif self._msg_managers[ch].empty():
                msg = f"No data on channel: {ch}"
                logging.warning(f"{websocket.id} - {msg}")
                err = MBotJSONError(msg)
                await websocket.send(err.encode())
            else:
                # Get the newest data and send it as bytes.
                res = self._msg_managers[ch].latest()
                message_staleness_us = time.time_ns() // 1000 - res.utime
                if self.discard_msgs > 0 and message_staleness_us > self.discard_msgs * 1E6:
                    # remove from the queue
                    msg = f"Data on channel {ch} is old."
                    logging.warning(f"Old data on channel: {ch} of staleness {message_staleness_us} us discarded")
                    self._msg_managers[ch].pop()
                    err = MBotJSONError(msg)
                    await websocket.send(err.encode())
                else:
                    await websocket.send(res.encode())
        elif request.type() == MBotMessageType.PUBLISH:
            try:
                # Publish the data sent over the websocket.
                pub_msg = type_utils.dict_to_lcm_type(request.data(), request.dtype())
                pub_msg.utime = time.time_ns() // 1000
                self._lcm.publish(request.channel(), pub_msg.encode())
            except type_utils.BadMessageError as e:
                # If the type or data is bad, send back an error message.
                msg = (f"Bad MBot publish. Bad message type ({request.dtype()}) or data (\"{request.data()}\"). "
                       f"AttributeError: {e}")
                logging.warning(f"{websocket.id} - {msg}")
                err = MBotJSONError(msg)
                await websocket.send(err.encode())
        elif request.type() == MBotMessageType.SUBSCRIBE:
            logging.debug(f"Websocket ID {websocket.id} - Subscribed to channel {request.channel()}")
            self._subscribe(websocket, request.channel())
        elif request.type() == MBotMessageType.UNSUBSCRIBE:
            logging.debug(f"Websocket ID {websocket.id} - Unsubscribed from channel {request.channel()}")
            self._unsubscribe(websocket, request.channel())

    async def handler(self, websocket):
        logging.debug(f"Websocket connected with ID: {websocket.id}")

        try:
            # Handle all incoming messages from the websocket.
            async for message in websocket:
                logging.debug(f"Message from WS {websocket.id}: {message}")
                await self.process_msg(websocket, message)
        except websockets.exceptions.ConnectionClosedOK:
            logging.debug(f"Websocket connection closed: {websocket.id}")
        except websockets.exceptions.ConnectionClosedError:
            logging.warning(f"Websocket closed with error: {websocket.id}")


async def main(args):
    logging.info(f"Reading configuration from: {args.config}")
    with open(args.config, 'r') as f:
        config = yaml.load(f, Loader=yaml.Loader)

    # Set the stop condition when receiving SIGTERM or SIGINT.
    loop = asyncio.get_running_loop()
    stop = asyncio.Future()
    loop.add_signal_handler(signal.SIGTERM, stop.set_result, None)
    loop.add_signal_handler(signal.SIGINT, stop.set_result, None)
    lcm_manager = MBotBridgeServer(config["lcm_address"], subs=config["subs"], hostfile=args.host_file, discard_msgs=args.discard_msgs)

    # Not awaiting the task will cause it to be stoped when the loop ends.
    asyncio.create_task(asyncio.to_thread(lcm_manager.lcm_loop))

    async with websockets.serve(
        lcm_manager.handler,
        host="",
        port=args.port,
        reuse_port=True,
    ):
        await stop
        lcm_manager.stop()

    logging.info("MBot Bridge exited cleanly.")


def load_args(conf="config/default.yml"):
    parser = argparse.ArgumentParser(description="MBot Bridge Server.")
    parser.add_argument("--config", type=str, default=conf, help="Configuration file.")
    parser.add_argument("--port", type=int, default=5005, help="Websocket port.")
    parser.add_argument("--log-file", type=str, default="mbot_bridge_server.log", help="Log file.")
    parser.add_argument("--log", type=str, default="INFO", help="Log level.")
    parser.add_argument("--max-log-size", type=int, default=2 * 1024 * 1024, help="Max log size.")
    parser.add_argument("--host-file", type=str, default="/etc/hostname", help="Hostname file.")
    parser.add_argument("--discard-msgs", type=float, default=-1, help="Discard stale msgs after X seconds.")

    args = parser.parse_args()

    # Turn the logging level into the correct form.
    numeric_level = getattr(logging, args.log.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f'Invalid log level: {args.log}')
    args.log = numeric_level

    return args


if __name__ == "__main__":
    import os
    import argparse
    from . import config
    from logging import handlers

    DEFAULT_CONFIG = os.path.join(config.__path__[0], 'default.yml')

    args = load_args(DEFAULT_CONFIG)

    # Setup logging.
    file_handler = handlers.RotatingFileHandler(args.log_file, maxBytes=args.max_log_size)
    logging.basicConfig(level=args.log,
                        handlers=[
                            file_handler,
                            logging.StreamHandler()  # Also print to terminal.
                        ],
                        format='%(asctime)s [%(name)s] [%(levelname)s] %(message)s',
                        datefmt='%m/%d/%Y %I:%M:%S %p')
    # Websocket messages are too noisy, make sure they aren't higher than warning.
    logging.getLogger("websockets").setLevel(logging.WARNING)

    asyncio.run(main(args))
