const worker = new Worker("oneline.js");
worker.onmessage = handleMessage;
let seq = 1;

function setImage(array) {
  let msg = {
    type: "setImage",
    seq: seq++,
    data: array,
  };
  worker.postMessage(msg);
}

function build(options) {
  let msg = {
    type: "build",
    seq: seq++,
    options: JSON.stringify(options),
  };
  worker.postMessage(msg);
}

function handleMessage(msg) {
  // console.log("Received message from webworker", msg);
  if(exports.onResult) {
    exports.onResult(msg.data);
  }
}

let exports = {
  worker: worker,
  setImage: setImage,
  build: build,
  onResult: undefined,
};

export default exports;

