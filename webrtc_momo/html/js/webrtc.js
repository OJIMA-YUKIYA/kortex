const remoteVideo = document.getElementById('remote_video');
const T2_Input = document.getElementById('T2_text');
const accel_Input = document.getElementById('accel_text');
const max_velocity_Input = document.getElementById('max_velocity_text');
const reverse_Input = document.getElementsByName('q1');
const leftright_progress = document.getElementById('leftright');

var ideal_velocity_logData = '';
var real_velocity_logData = '';
var log_latch = false;

remoteVideo.controls = true;
let peerConnection = null;
let dataChannel = null;
let candidates = [];
let hasReceivedSdp = false;
// iceServer を定義
const iceServers = [{ 'urls': 'stun:stun.l.google.com:19302' }];
// peer connection の 設定
const peerConnectionConfig = {
  'iceServers': iceServers
};


const isSSL = location.protocol === 'https:';
const wsProtocol = isSSL ? 'wss://' : 'ws://';
const wsUrl = wsProtocol + location.host + '/ws';
const ws = new WebSocket(wsUrl);
ws.onopen = onWsOpen.bind();
ws.onerror = onWsError.bind();
ws.onmessage = onWsMessage.bind();

function onWsError(error){
  console.error('ws onerror() ERROR:', error);
}

function onWsOpen(event) {
  console.log('ws open()');
}
function onWsMessage(event) {
  console.log('ws onmessage() data:', event.data);
  const message = JSON.parse(event.data);
  if (message.type === 'offer') {
    console.log('Received offer ...');
    const offer = new RTCSessionDescription(message);
    console.log('offer: ', offer);
    setOffer(offer);
  }
  else if (message.type === 'answer') {
    console.log('Received answer ...');
    const answer = new RTCSessionDescription(message);
    console.log('answer: ', answer);
    setAnswer(answer);
  }
  else if (message.type === 'candidate') {
    console.log('Received ICE candidate ...');
    const candidate = new RTCIceCandidate(message.ice);
    console.log('candidate: ', candidate);
    if (hasReceivedSdp) {
      addIceCandidate(candidate);
    } else {
      candidates.push(candidate);
    }
  }
  else if (message.type === 'close') {
    console.log('peer connection is closed ...');
  }
}

function connect() {
  console.group();
  if (!peerConnection) {
    console.log('make Offer');
    makeOffer();
  }
  else {
    console.warn('peer connection already exists.');
  }
  console.groupEnd();
}

function disconnect() {
  console.group();
  if (peerConnection) {
    if (peerConnection.iceConnectionState !== 'closed') {
      peerConnection.close();
      peerConnection = null;
      if (ws && ws.readyState === 1) {
        const message = JSON.stringify({ type: 'close' });
        ws.send(message);
      }
      console.log('sending close message');
      cleanupVideoElement(remoteVideo);
      return;
    }
  }
  console.log('peerConnection is closed.');
  console.groupEnd();
}

function drainCandidate() {
  hasReceivedSdp = true;
  candidates.forEach((candidate) => {
    addIceCandidate(candidate);
  });
  candidates = [];
}

function addIceCandidate(candidate) {
  if (peerConnection) {
    peerConnection.addIceCandidate(candidate);
  }
  else {
    console.error('PeerConnection does not exist!');
  }
}

function sendIceCandidate(candidate) {
  console.log('---sending ICE candidate ---');
  const message = JSON.stringify({ type: 'candidate', ice: candidate });
  console.log('sending candidate=' + message);
  ws.send(message);
}

function playVideo(element, stream) {
  element.srcObject = stream;
}

function prepareNewConnection() {
  const peer = new RTCPeerConnection(peerConnectionConfig);
  dataChannel = peer.createDataChannel("serial");
  if ('ontrack' in peer) {
    if (isSafari()) {
      let tracks = [];
      peer.ontrack = (event) => {
        console.log('-- peer.ontrack()');
        tracks.push(event.track)
        // safari で動作させるために、ontrack が発火するたびに MediaStream を作成する
        let mediaStream = new MediaStream(tracks);
        playVideo(remoteVideo, mediaStream);
      };
    }
    else {
      let mediaStream = new MediaStream();
      playVideo(remoteVideo, mediaStream);
      peer.ontrack = (event) => {
        console.log('-- peer.ontrack()');
        mediaStream.addTrack(event.track);
      };
    }
  }
  else {
    peer.onaddstream = (event) => {
      console.log('-- peer.onaddstream()');
      playVideo(remoteVideo, event.stream);
    };
  }

  peer.onicecandidate = (event) => {
    console.log('-- peer.onicecandidate()');
    if (event.candidate) {
      console.log(event.candidate);
      sendIceCandidate(event.candidate);
    } else {
      console.log('empty ice event');
    }
  };

  peer.oniceconnectionstatechange = () => {
    console.log('-- peer.oniceconnectionstatechange()');
    console.log('ICE connection Status has changed to ' + peer.iceConnectionState);
    switch (peer.iceConnectionState) {
      case 'closed':
      case 'failed':
      case 'disconnected':
        break;
    }
  };
  peer.addTransceiver('video', {direction: 'recvonly'});
  peer.addTransceiver('audio', {direction: 'recvonly'});

  dataChannel.onmessage = function (event) {
    // console.log(event.data.byteLength);

    if (event.data.byteLength == 12) {

      let vel_time = (new Int32Array([new Uint8Array(event.data)[0] << 24])[0] + new Int32Array([new Uint8Array(event.data)[1] << 16])[0] + new Int32Array([new Uint8Array(event.data)[2] << 8])[0] + new Int32Array([ new Uint8Array(event.data)[3]])[0] )/1000.0 ;
      let lin_vel = (new Int32Array([new Uint8Array(event.data)[4] << 24])[0] + new Int32Array([ new Uint8Array(event.data)[5] << 16 ])[0] + new Int32Array([new Uint8Array(event.data)[6] << 8])[0] + new Int32Array([ new Uint8Array(event.data)[7]])[0] )/10000.0 ;
      let ang_vel = (new Int32Array([new Uint8Array(event.data)[8] << 24])[0] + new Int32Array([ new Uint8Array(event.data)[9] << 16 ])[0] + new Int32Array([new Uint8Array(event.data)[10] << 8])[0] + new Int32Array([ new Uint8Array(event.data)[11]])[0] )/10000.0 ;

      document.getElementById("sgss").innerHTML = '時刻(s) ' + vel_time + '\n並進速度(m/s) ' + lin_vel + '\n旋回速度(deg/s)' + ang_vel;
      if (log_latch) {
        real_velocity_logData = real_velocity_logData + vel_time + ' ' + lin_vel + ' ' + ang_vel + '\n';
      }
    }
  };

  return peer;
}

function browser() {
  const ua = window.navigator.userAgent.toLocaleLowerCase();
  if (ua.indexOf('edge') !== -1) {
    return 'edge';
  }
  else if (ua.indexOf('chrome')  !== -1 && ua.indexOf('edge') === -1) {
    return 'chrome';
  }
  else if (ua.indexOf('safari')  !== -1 && ua.indexOf('chrome') === -1) {
    return 'safari';
  }
  else if (ua.indexOf('opera')   !== -1) {
    return 'opera';
  }
  else if (ua.indexOf('firefox') !== -1) {
    return 'firefox';
  }
  return ;
}

function isSafari() {
  return browser() === 'safari';
}

function sendSdp(sessionDescription) {
  console.log('---sending sdp ---');
  const message = JSON.stringify(sessionDescription);
  console.log('sending SDP=' + message);
  ws.send(message);
}

async function makeOffer() {
  peerConnection = prepareNewConnection();
  try {
    const sessionDescription = await peerConnection.createOffer({
      'offerToReceiveAudio': true,
      'offerToReceiveVideo': true
    })
    console.log('createOffer() success in promise, SDP=', sessionDescription.sdp);
    switch (document.getElementById('codec').value) {
      case 'H264':
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
        break;
      case 'VP8':
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
        break;
      case 'VP9':
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
        break;
      case 'AV1':
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
        break;
    }
    await peerConnection.setLocalDescription(sessionDescription);
    console.log('setLocalDescription() success in promise');
    sendSdp(peerConnection.localDescription);
  } catch (error) {
    console.error('makeOffer() ERROR:', error);
  }
}

async function makeAnswer() {
  console.log('sending Answer. Creating remote session description...');
  if (!peerConnection) {
    console.error('peerConnection DOES NOT exist!');
    return;
  }
  try {
    const sessionDescription = await peerConnection.createAnswer();
    console.log('createAnswer() success in promise');
    await peerConnection.setLocalDescription(sessionDescription);
    console.log('setLocalDescription() success in promise');
    sendSdp(peerConnection.localDescription);
    drainCandidate();
  } catch (error) {
    console.error('makeAnswer() ERROR:', error);
  }
}

// offer sdp を生成する
function setOffer(sessionDescription) {
  if (peerConnection) {
    console.error('peerConnection already exists!');
  }
  const peerConnection = prepareNewConnection();
  peerConnection.onnegotiationneeded = async function () {
    try{
      await peerConnection.setRemoteDescription(sessionDescription);
      console.log('setRemoteDescription(offer) success in promise');
      makeAnswer();
    }catch(error) {
      console.error('setRemoteDescription(offer) ERROR: ', error);
    }
  }
}

async function setAnswer(sessionDescription) {
  if (!peerConnection) {
    console.error('peerConnection DOES NOT exist!');
    return;
  }
  try {
    await peerConnection.setRemoteDescription(sessionDescription);
    console.log('setRemoteDescription(answer) success in promise');
    drainCandidate();
  } catch(error) {
    console.error('setRemoteDescription(answer) ERROR: ', error);
  }
}

function cleanupVideoElement(element) {
  element.pause();
  element.srcObject = null;
}


/* getOffer() function is currently unused.
function getOffer() {
  initiator = false;
  createPeerConnection();
  sendXHR(
    ".GetOffer",
    JSON.stringify(peer_connection.localDescription),
    function (respnse) {
      peer_connection.setRemoteDescription(
        new RTCSessionDescription(respnse),
        function () {
          peer_connection.createAnswer(
            function (answer) {
              peer_connection.setLocalDescription(answer);
            }, function (e) { });
        }, function (e) {
          console.error(e);
        });
    }, true);
}
*/

// Stack Overflow より引用: https://stackoverflow.com/a/52760103
// https://stackoverflow.com/questions/52738290/how-to-remove-video-codecs-in-webrtc-sdp
function removeCodec(orgsdp, codec) {
  const internalFunc = (sdp) => {
    const codecre = new RegExp('(a=rtpmap:(\\d*) ' + codec + '\/90000\\r\\n)');
    const rtpmaps = sdp.match(codecre);
    if (rtpmaps == null || rtpmaps.length <= 2) {
      return sdp;
    }
    const rtpmap = rtpmaps[2];
    let modsdp = sdp.replace(codecre, "");

    const rtcpre = new RegExp('(a=rtcp-fb:' + rtpmap + '.*\r\n)', 'g');
    modsdp = modsdp.replace(rtcpre, "");

    const fmtpre = new RegExp('(a=fmtp:' + rtpmap + '.*\r\n)', 'g');
    modsdp = modsdp.replace(fmtpre, "");

    const aptpre = new RegExp('(a=fmtp:(\\d*) apt=' + rtpmap + '\\r\\n)');
    const aptmaps = modsdp.match(aptpre);
    let fmtpmap = "";
    if (aptmaps != null && aptmaps.length >= 3) {
      fmtpmap = aptmaps[2];
      modsdp = modsdp.replace(aptpre, "");

      const rtppre = new RegExp('(a=rtpmap:' + fmtpmap + '.*\r\n)', 'g');
      modsdp = modsdp.replace(rtppre, "");
    }

    let videore = /(m=video.*\r\n)/;
    const videolines = modsdp.match(videore);
    if (videolines != null) {
      //If many m=video are found in SDP, this program doesn't work.
      let videoline = videolines[0].substring(0, videolines[0].length - 2);
      const videoelems = videoline.split(" ");
      let modvideoline = videoelems[0];
      videoelems.forEach((videoelem, index) => {
        if (index === 0) return;
        if (videoelem == rtpmap || videoelem == fmtpmap) {
          return;
        }
        modvideoline += " " + videoelem;
      })
      modvideoline += "\r\n";
      modsdp = modsdp.replace(videore, modvideoline);
    }
    return internalFunc(modsdp);
  }
  return internalFunc(orgsdp);
}

function play() {
  remoteVideo.play();
}

function cal_velocity_plan() {
    let a = parseFloat(accel_Input.value);
    let vel_limit = parseFloat(max_velocity_Input.value);
    let T2 = parseFloat(T2_Input.value);
    let T1 = vel_limit / a;
    let T3 = vel_limit / a;
    let x =  (T2 + (T1 + T2 + T3)) * vel_limit / 2.0;
    document.getElementById("result_velocity_plan").innerHTML =
      "合計時間: " + (T1 + T2 + T3).toFixed(6) + " (s)\n"
    + "      x: " + x.toFixed(6) + " (m)\n"
    + "     T1: " + T1.toFixed(6) + " (s)\n"
    + "     T2: " + T2.toFixed(6) + " (s)\n"
    + "     T3: " + T3.toFixed(6) + " (s)";
}

const sleep = waitTime => new Promise( resolve => setTimeout(resolve, waitTime) );

const sendDataChannel = async function() {
    // console.log("hello");
    // if (accel_Input.value ==  || max_velocity_Input.value == NULL) {
    //     let target = document.getElementById("warning");
    //     target.innerHTML = "両方入力してください。";
    //     accel_Input.value = "";
    //     max_velocity_Input.value = "";
    //     return;
    // }
    // let target = document.getElementById("warning");
    // target.innerHTML = "";

    // let textData = "acce" + T2_Input.value + "," + accel_Input.value + "," + max_velocity_Input.value + ",";
    // if (reverse_Input[0].checked) {
    //     textData = textData + 0;
    // }
    // else {
    //     textData = textData + 1;
    // }
    // console.log("send: " + textData);
    // if (textData.length == 0) {
    //     return;
    // }
    // if (dataChannel == null || dataChannel.readyState != "open") {
    //     // console.log("hello");
    //     return;
    // }
    // dataChannel.send(new TextEncoder().encode(textData));


    if (document.getElementsByName("q2")[0].checked) {
        dataChannel.send( new Uint8Array([ 0x01, 0x00, 0x00, document.getElementById("offset_input").value*100 ]) );
    }
    else if (document.getElementsByName("q2")[1].checked) {
        dataChannel.send( new Uint8Array([ 0x02, 0x00, 0x00, document.getElementById("gain_input").value*100 ]) );
    }
    else if (document.getElementsByName("q2")[2].checked) {
        dataChannel.send( new Uint8Array([ 0x03, 0x00, 0x00, 0x00 ]) );
    }

    await sleep(10);

    const header = 0xa0 + (document.getElementById("count_input").value & 0x0f);

    // if (reverse_Input[0].checked) {
        dataChannel.send( new Uint8Array([ header, T2_Input.value*2, accel_Input.value*20, max_velocity_Input.value*20 ]));
    // }
    // else {
        // dataChannel.send( new Uint8Array([ 0xab, T2_Input.value*2, accel_Input.value*20, max_velocity_Input.value*20 ]));
    // }
}

function d_left() {
    dataChannel.send(new Uint8Array([ 0xd1, 0, 127, 0 ]));
}

function d_right() {
    dataChannel.send(new Uint8Array([ 0xd1, 0, -127, 0 ]));
}

function quit_accel_cmd() {
    dataChannel.send(new Uint8Array([0x99, 0x99, 0x99, 0x99]));
}

function handleTargetVelDownload() {
    ideal_velocity_logData = '#理想の速度の記録 [加速度: ' + accel_Input.value + ', 最高速度: ' + max_velocity_Input.value + ', T2: ' + T2_Input.value + ']\n#時刻(s),両輪(m/s)\n' + ideal_velocity_logData;
    let blob = new Blob([ideal_velocity_logData], {"type": "text/plain"});

    if (window.navigator.msSaveBlob) {
        window.navigator.msSaveBlob(blob, "target_velocity.log");
        window.navigator.msSaveOrOpenBlob(blob, "target_velocity.log");
    } else {
        document.getElementById("download1").href = window.URL.createObjectURL(blob);
    }
}

function handleActualVelDownload() {
    const blob = new Blob([real_velocity_logData], {"type": "text/plain"});
    let file_name = "actual_velocity_a_" + accel_Input.value + "_v_" + max_velocity_Input.value + "_T2_" + T2_Input.value;
    if (document.getElementsByName('q2')[0].checked) {
        file_name = file_name + "_offset_" + document.getElementById("offset_input").value + '.txt';
    }
    else if (document.getElementsByName('q2')[1].checked) {
        file_name = file_name + "_gain_" + document.getElementById("gain_input").value + '.txt';
    }
    else if (document.getElementsByName('q2')[2].checked) {
        file_name = file_name + '.txt';
    }

    // const url = window.URL.createObjectURL(blob);
    // console.log(url);
    // const a = document.createElement("a");
    // document.body.appendChild(a);
    // a.download = file_name;
    // a.href = url;
    // a.click();
    // a.remove();
    // setTimeout(() => {
    //     a.remove();
    //     window.URL.revokeObjectURL(url);
    //     console.log("hello");
    // }, 1000);

    document.getElementById("download2").download = file_name;
    // if (window.navigator.msSaveBlob) {
    //     window.navigator.msSaveBlob(blob, file_name);
    //     window.navigator.msSaveOrOpenBlob(blob, file_name);
        // console.log("hello");
    // } else {
        document.getElementById("download2").href = window.URL.createObjectURL(blob);
    // }

    // document.getElementById("download2").remove();
    // const a = document.createElement("a");
    // a.id = "download2";
    // a.href = '#';
    // a.onclick="handleActualVelDownload()";
    // a.innerHTML = "ダウンロード";
    // document.getElementById("download2_wrapper").appendChild(a);

    // setTimeout(() => {
    //     window.URL.revokeObjectURL(document.getElementById("download2").href);
    //     document.getElementById('download2').removeAttribute('download');
    //     document.getElementById('download2').removeAttribute('href');
    //     console.log("hello");
    // }, 500);
}

function startLog() {
    ideal_velocity_logData = '';
    real_velocity_logData = '#accel(m/s^2) ' + accel_Input.value + ' max_vel(m/s) ' + max_velocity_Input.value + ' max_vel_time(s) ' + T2_Input.value;
    if (document.getElementsByName('q2')[0].checked) {
        real_velocity_logData = real_velocity_logData + ' offset ' + document.getElementById("offset_input").value + '\n#時刻(s) 実際の速度(m/s)\n';
    }
    else if (document.getElementsByName('q2')[1].checked) {
        real_velocity_logData = real_velocity_logData + ' gain ' + document.getElementById("gain_input").value + '\n#時刻(s) 実際の速度(m/s)\n';
    }
    else if (document.getElementsByName('q2')[2].checked) {
        real_velocity_logData = real_velocity_logData + '\n#時刻(s) 実際の速度(m/s)\n';
    }
    let target1 = document.getElementById('logstartbutton');
    if (target1.value == '記録中') {
        return;
    }
    target1.value = '記録中';
    let target2 = document.getElementById('logendbutton');
    target2.value = '記録終了';
    log_latch = true;
}

function endLog() {
    log_latch = false;
    let target1 = document.getElementById('logstartbutton');
    target1.value = '記録開始';
    let target2 = document.getElementById('logendbutton');
    target2.value = '記録完了';
}



// joystick

window.addEventListener("gamepadconnected", function(e) {
    // gp = navigator.getGamepads()[e.gamepad.index];
    console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
    setInterval(gameLoop, 1000.0/20.0);
    gameLoop();
});

// let start = true;

function gameLoop() {
    let gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads : []);
    let gp = gamepads[0];
    if (gp != null) {
        let side = gp.buttons[6].value - gp.buttons[7].value;
        let ang = -gp.axes[0];
        let lin = -gp.axes[3];
        // let ang = -gp.axes[0] * Math.abs(gp.axes[0]);
        // let lin = -gp.axes[3] * Math.abs(gp.axes[3]);
        document.getElementById('leftright').value = 127*ang;
        document.getElementById('leftright_out').innerHTML = (127*ang).toFixed(0);
        document.getElementById('frontrear').value = 127*lin;
        document.getElementById('frontrear_out').innerHTML = (127*lin).toFixed(0);
        document.getElementById('side').value = 127*side;
        document.getElementById('side_out').innerHTML = (127*side).toFixed(0);
        if (dataChannel != null) {
            // if (gp.buttons[1].value) {
            //     start = true;
            //     dataChannel.send(new Uint8Array([0x11, 0x11, 0x11, 0x11]));
            //     return;
            // }
            // if (gp.buttons[0].value) {
            //     start = false;
            //     dataChannel.send(new Uint8Array([0x99, 0x99, 0x99, 0x99]));
            //     return;
            // }
            if (document.getElementsByName("q3")[1].checked) {
                ang = 127*ang;
                lin = 127*lin;
                side = 127*side;
                let send_value = new Uint8Array([0x43, side, ang, lin]);
                dataChannel.send(send_value);
                return;
            }
        }
    }
}
// function gameLoop() {
//     let gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads : []);
//     let gp = gamepads[0];
//     if (gp != null) {
//         let ang = -gp.axes[0];
//         let lin = -gp.axes[3];
//         document.getElementById('leftright').value = 50*ang;
//         document.getElementById('leftright_out').innerHTML = 50*ang.toFixed(3);
//         document.getElementById('frontrear').value = lin;
//         document.getElementById('frontrear_out').innerHTML = lin.toFixed(3);
//         if (dataChannel != null) {
//             // dataChannel.send(new TextEncoder().encode("jyja" + ang.toFixed(3) + "," + lin.toFixed(3) + "\n"));
//             // const buffer = new ArrayBuffer(1);
//             // console.log(buffer);
//             ang = 127*ang;
//             lin = 127*lin;
//             ang = (ang << 8) & 0x0000ff00;
//             lin = lin & 0x000000ff;
//             let send_value = new Int32Array([0x43000000 | ang | lin]);
//             dataChannel.send(send_value);
//         }
//     }
// }
