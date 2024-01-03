# MBot Bridge

Server and API to bridge mbot functionality with student code.

Dependencies:
* Python 3 (tested on 3.10)
* LCM 1.4+ (tested on 1.5)
* MBot messages for Python

## Install Instructions

```bash
pip install -e .
```

## Usage

To run the server, do:
```bash
python -m mbot_bridge.server [--config [PATH/TO/CONFIG]]
```
The `--config` argument is optional and will default to `src/mbot_bridge/config/default.yml`.

## C++ API

To use the C++ API, first install the websocket library at the latest release version:
```bash
wget https://github.com/zaphoyd/websocketpp/archive/refs/tags/0.8.2.tar.gz
tar -xzf 0.8.2.tar.gz
cd websocketpp-0.8.2/
mkdir build && cd build
cmake ..
make
sudo make install
```
Then, compile the C++ API:
```bash
cd mbot_bridge
python setup.py build_ext
sudo python setup.py install_ext
```
A test script is available at:
```bash
./build/mbot_cpp_test
```

## JavaScript API

The JavaScript API can be installed using NPM as follows:
```bash
cd mbot_js
npm install
npm run build
```
Then include the bundled script `mbot_js/dist/main.js` where you want to use the API. To create a MBot object, do:
```javascript
const mbot = new MBotAPI.MBot(mbotIP);
```
where `mbotIP` is the IP address of the mbot.

### Usage

To test the JS API, a test script is available in the `test` directory. To use it, start an HTTP server in the root of this repo:
```bash
python -m http.server
```
Then navigate to `http://[HOST_IP]:8000` in a browser. If running locally, use the IP `localhost`.
