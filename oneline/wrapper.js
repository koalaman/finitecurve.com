function myInit() {
  // console.log(Module);
  let oneline = new Module.OneLine();

  function getMessage(event) {
    let msg = event.data;
    // console.log("Received message from main", msg);
    switch(msg.type) {
      case 'setImage':
	oneline.setImage(msg.data);
	break;
      case 'build':
	build(msg);
	break;
      default:
	console.error('Unknown message type: ' + msg.type);
	console.log(msg);
    }
  }

  function build(msg) {
    oneline.setOptions(msg.options);
    let success = oneline.build();
    let result = {
      seq: msg.seq,
      success: success,
      result: oneline.getResult(),
      error: oneline.getError(),
      width: oneline.getWidth(),
      height: oneline.getHeight(),
      lineLength: oneline.getLineDistance(),
    };
    postMessage(result);
  }

  addEventListener('message', getMessage);
}

Module['onRuntimeInitialized'] = myInit;

