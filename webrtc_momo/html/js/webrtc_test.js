const debug = true;
// const sora = Sora.connection("wss://207-148-92-89.stable.sora.sora-labo.shiguredo.app/signaling", debug);
const sora = Sora.connection("wss://sora.ikeilabsora.0am.jp/signaling", debug);
// const channelId = "OJIMA-YUKIYA@sora-devtools";
const channelId = "twincam-right";
// const metadata = {
//     "signaling_key": "0mKFzDghLJNL7bmqa99hj4pp13IGaG_o4SHWdHoIKMzffpyZwQmo5dOIVi_9QBZ_",
// };
const options = {
    multistream: true,
    video: false,
    audio: true,
    dataChannelSignaling: true,
    dataChannels: [
        {
            label: "#sora-devtools",
            direction: "sendrecv"
        }
    ]
};
let recvonly = sora.recvonly(channelId, null, options);


const remoteVideo = document.getElementById('remote_video');
// const T2_Input = document.getElementById('T2_text');
// const accel_Input = document.getElementById('accel_text');
// const max_velocity_Input = document.getElementById('max_velocity_text');
// const reverse_Input = document.getElementsByName('q1');
// const leftright_progress = document.getElementById('leftright');

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
        // tracks.push(event.track)
        // safari で動作させるために、ontrack が発火するたびに MediaStream を作成する
        // let mediaStream = new MediaStream(tracks);
        // playVideo(remoteVideo, mediaStream);
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
      if (event.data.byteLength == 19) {
        recvonly.sendMessage('#sora-devtools', event.data);
        let vel_time = (new Int32Array([new Uint8Array(event.data)[0] << 24])[0] + new Int32Array([new Uint8Array(event.data)[1] << 16])[0] + new Int32Array([new Uint8Array(event.data)[2] << 8])[0] + new Int32Array([ new Uint8Array(event.data)[3]])[0] )/1000.0;
        let lin_vel = (new Int32Array([new Uint8Array(event.data)[4] << 24])[0] + new Int32Array([ new Uint8Array(event.data)[5] << 16 ])[0] + new Int32Array([new Uint8Array(event.data)[6] << 8])[0] + new Int32Array([ new Uint8Array(event.data)[7]])[0] )/10000.0;
        let ang_vel = (new Int32Array([new Uint8Array(event.data)[8] << 24])[0] + new Int32Array([ new Uint8Array(event.data)[9] << 16 ])[0] + new Int32Array([new Uint8Array(event.data)[10] << 8])[0] + new Int32Array([ new Uint8Array(event.data)[11]])[0] )/10000.0;
        let latch = new Uint8Array(event.data)[12];
        let turn_position = (new Int16Array([new Uint8Array(event.data)[13] << 8])[0] + new Int16Array([ new Uint8Array(event.data)[14]])[0] )/100.0;
        let position_x = (new Int16Array([new Uint8Array(event.data)[15] << 8])[0] + new Int16Array([ new Uint8Array(event.data)[16]])[0] )/100.0;
        let position_z = (new Int16Array([new Uint8Array(event.data)[17] << 8])[0] + new Int16Array([ new Uint8Array(event.data)[18]])[0] )/100.0;

        document.getElementById("sgss").innerHTML = 'latch ' + latch + '\nturn position(deg) ' + turn_position + '\nposition x(m) ' + position_x + '\nposition z(m) ' + position_z  + '\n時刻(s) ' + vel_time + '\n並進速度(m/s) ' + lin_vel + '\n旋回速度(deg/s)' + ang_vel;
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
  console.log('hello!!!!!!!!!!!!');
  try {
    const sessionDescription = await peerConnection.createOffer({
      'offerToReceiveAudio': false,
      'offerToReceiveVideo': false
    })
    console.log('createOffer() success in promise, SDP=', sessionDescription.sdp);
    // switch (document.getElementById('codec').value) {
    //   case 'H264':
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
    //     break;
    //   case 'VP8':
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
    //     break;
    //   case 'VP9':
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
    //     break;
    //   case 'AV1':
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
    //     sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
    //     break;
    // }
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

// function play() {
//   remoteVideo.play();
// }

// function cal_velocity_plan() {
//     let a = parseFloat(accel_Input.value);
//     let vel_limit = parseFloat(max_velocity_Input.value);
//     let T2 = parseFloat(T2_Input.value);
//     let T1 = vel_limit / a;
//     let T3 = vel_limit / a;
//     let x =  (T2 + (T1 + T2 + T3)) * vel_limit / 2.0;
//     document.getElementById("result_velocity_plan").innerHTML =
//       "total_time: " + (T1 + T2 + T3).toFixed(6) + " (s)\n"
//     + "         x: " + x.toFixed(6) + " (m)\n"
//     + "        T1: " + T1.toFixed(6) + " (s)\n"
//     + "        T2: " + T2.toFixed(6) + " (s)\n"
//     + "        T3: " + T3.toFixed(6) + " (s)";
// }

// function sendDataChannel() {
//     // console.log("hello");
//     // if (accel_Input.value ==  || max_velocity_Input.value == NULL) {
//     //     let target = document.getElementById("warning");
//     //     target.innerHTML = "両方入力してください。";
//     //     accel_Input.value = "";
//     //     max_velocity_Input.value = "";
//     //     return;
//     // }
//     // let target = document.getElementById("warning");
//     // target.innerHTML = "";
//     // let textData = "acce" + T2_Input.value + "," + accel_Input.value + "," + max_velocity_Input.value + ",";
//     if (reverse_Input[0].checked) {
//         textData = textData + 0;
//     }
//     else {
//         textData = textData + 1;
//     }
//     // console.log("send: " + textData);
//     if (textData.length == 0) {
//         return;
//     }
//     if (dataChannel == null || dataChannel.readyState != "open") {
//         // console.log("hello");
//         return;
//     }
//     dataChannel.send(new TextEncoder().encode(textData));
//     // accel_Input.value = "";
//     // max_velocity_Input.value = "";
// }

// function quit_accel_cmd() {
//     dataChannel.send(new TextEncoder().encode("quit"));
// }

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
    real_velocity_logData = '#計測した速度の記録\n#時刻(s),左右平均(m/s),左車輪(m/s),右車輪(m/s)\n' + real_velocity_logData;
    let blob = new Blob([real_velocity_logData], {"type": "text/plain"});

    if (window.navigator.msSaveBlob) {
        window.navigator.msSaveBlob(blob, "actual_velocity.log");
        window.navigator.msSaveOrOpenBlob(blob, "actual_velocity.log");
    } else {
        document.getElementById("download2").href = window.URL.createObjectURL(blob);
    }
}

function startLog() {
    ideal_velocity_logData = '';
    real_velocity_logData = '';
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

// window.addEventListener("gamepadconnected", function(e) {
//     // gp = navigator.getGamepads()[e.gamepad.index];
//     console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
//     e.gamepad.index, e.gamepad.id,
//     e.gamepad.buttons.length, e.gamepad.axes.length);
//     setInterval(gameLoop, 1000.0/50.0);
//     gameLoop();
// });
// function gameLoop() {
//     let gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads : []);
//     let gp = gamepads[0];
//     if (gp != null) {
//         let ang = -50*gp.axes[0];
//         let lin = -0.8*gp.axes[3];
//         document.getElementById('leftright').value = ang;
//         document.getElementById('leftright_out').innerHTML = ang.toFixed(3);
//         document.getElementById('frontrear').value = lin;
//         document.getElementById('frontrear_out').innerHTML = lin.toFixed(3);
//         if (dataChannel != null) {
//             dataChannel.send(new TextEncoder().encode("jyja" + ang.toFixed(3) + "," + lin.toFixed(3) + "\n"));
//         }
//     }
// }
