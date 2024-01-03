import { MBotMessageType, MBotJSONMessage } from "./mbot_json_msgs.js";
import config from "./lcm_config.js";


class MBot {
  constructor(hostname = "localhost", port = 5005) {
    this.address = "ws://" + hostname + ":" + port;
    this.ws_subs = {};
  }

  async _read(ch) {
    let msg = new MBotJSONMessage(null, ch, null, MBotMessageType.REQUEST);

    let promise = new Promise((resolve, reject) => {
      const websocket = new WebSocket(this.address);

      websocket.onopen = (event) => {
        websocket.send(msg.encode());
      };

      websocket.onmessage = (event) => {
        let res = new MBotJSONMessage()
        res.decode(event.data);
        websocket.close();

        // Check for error from the server.
        if (res.rtype === MBotMessageType.ERROR) {
          console.warn("MBot API Error:", res.data);
        }
        else if (res.rtype !== MBotMessageType.RESPONSE) {
          // Check if an unknown error occured.
          console.warn("MBot API Error: Can't parse response:", res);
        }

        resolve(res);
      };
    });

    return promise;

  }

  _publish(data, ch, dtype) {
    let msg = new MBotJSONMessage(data, ch, dtype, MBotMessageType.PUBLISH);
    const websocket = new WebSocket(this.address);

    websocket.onopen = (event) => {
      websocket.send(msg.encode());
      websocket.close();
    };
  }

  subscribe(ch, cb) {
    let msg = new MBotJSONMessage(null, ch, null, MBotMessageType.SUBSCRIBE);
    const websocket = new WebSocket(this.address);
    this.ws_subs[ch] = websocket;

    websocket.onopen = (event) => {
      websocket.send(msg.encode());
    };

    websocket.onmessage = (event) => {
      cb(event.data);
    };
  }

  unsubscribe(ch) {
    if (this.ws_subs[ch] === undefined || this.ws_subs[ch] === null) return;

    let msg = new MBotJSONMessage(null, ch, null, MBotMessageType.UNSUBSCRIBE);
    this.ws_subs[ch].send(msg.encode());
    this.ws_subs[ch].close();
    this.ws_subs[ch] = null;
  }

  drive(vx, vy, wz) {
    let data = { "vx": vx, "vy": vy, "wz": wz };
    this._publish(data, config.MOTOR_VEL_CMD.channel, config.MOTOR_VEL_CMD.dtype)
  }

  async readOdometry(odomCallback) {
    let waitForData = this._read(config.ODOMETRY.channel);
    waitForData.then((val) => {
      let odom = [];
      if (val.rtype === MBotMessageType.RESPONSE) {
        odom = [val.data.x, val.data.y, val.data.theta];
      }
      odomCallback(odom);
    });
    await waitForData;
  }
}

export { MBot };
