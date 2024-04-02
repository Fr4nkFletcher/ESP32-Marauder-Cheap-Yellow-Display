document.addEventListener('DOMContentLoaded', () => {
    const connectButton = document.getElementById('butConnect');
    const disconnectButton = document.getElementById('butDisconnect');
    const baudRateSelect = document.getElementById('baudRate');
    const terminalContainer = document.getElementById('terminalContainer');
    const sendButton = document.getElementById('butSend');
    const inputField = document.getElementById('terminalInput');
    const statusDisplay = document.createElement('div');
    const textDecoder = new TextDecoder();
    const textEncoder = new TextEncoder(); 
    terminalContainer.appendChild(statusDisplay);

    let port;
    let writer;
    let globalReader = null;
    let device_connected = false;
    let readLoopExitSignalResolver;
    let readLoopExitSignal = new Promise(resolve => {
    readLoopExitSignalResolver = resolve;
});

    async function updateStatus(message) {
        statusDisplay.textContent = message; 
    }

    async function connect() {
        if ('serial' in navigator) {
            updateStatus("Serial is supported by navigator.");
            if (port) {
                updateStatus("Port is already open. Please disconnect before trying to connect again.");
                return; 
            }
            try {
                port = await navigator.serial.requestPort();
                const baudRate = getBaudRate(); 
                await port.open({ baudRate });

                await port.setSignals({ dataTerminalReady: true, requestToSend: true });
                device_connected = true;
                readLoop();
                updateStatus("Connected successfully.");
                connectButton.style.display = 'none';
                disconnectButton.style.display = 'inline-block';
                disconnectButton.disabled = false;
                connectButton.disabled = true;
            } catch (err) {
                updateStatus(`Error accessing the serial port: ${err}`);
            }
        } else {
            updateStatus('Web Serial API not supported by your browser.');
        }
    }
    
    function getBaudRate() {
        let baudRate = document.getElementById('baudRate').value;
        if (baudRate === "custom") {
            baudRate = document.getElementById('baudRateCustom').value;
        }
        return parseInt(baudRate, 10) || 9600;
    }

    async function readLoop() {
        readLoopActive = true;
        globalReader = port.readable.getReader();
        try {
            while (port.readable && device_connected) {
                const { value, done } = await globalReader.read();
                if (done || !device_connected) {
                    break;
                }
                const textChunk = textDecoder.decode(value);
                displayData(textChunk);
            }
        } catch (error) {
            console.error('Read error:', error);
            updateStatus('Read error: ' + error);
        } finally {
            globalReader.releaseLock();
        }
        readLoopActive = false;
        readLoopExitSignalResolver();
    }

    function displayData(data) {
        const filteredData = data.replace(/\u0007/g, '');
    
        let outputContainer = document.querySelector('.terminal-output');
        let pre = outputContainer.querySelector('pre');
        if (!pre) {
            pre = document.createElement('pre');
            outputContainer.appendChild(pre);
        }
        pre.textContent += filteredData;
        outputContainer.scrollTop = outputContainer.scrollHeight;
    }

    async function send(data = '') {
        if (!port || !port.writable) {
            console.error('The port is not writable or not connected.');
            updateStatus('The port is not writable or not connected.');
            return;
        }
    
        const writer = port.writable.getWriter();
    
        try {
            const eolOption = document.getElementById('endOfLine').value;
            let formattedData = typeof data === 'string' ? data : '';
            
            switch (eolOption) {
                case 'cr':
                    formattedData += '\r';
                    break;
                case 'lf':
                    formattedData += '\n';
                    break;
                case 'crlf':
                    formattedData += '\r\n';
                    break;
            }
            
            const sendData = textEncoder.encode(formattedData);
            await writer.write(sendData);
            updateStatus(`Data sent: ${formattedData}`);
        } catch (error) {
            console.error('Write error:', error);
            updateStatus(`Write error: ${error}`);
        } finally {
            writer.releaseLock();
        }
    }

    inputField.addEventListener('keydown', async (event) => {
        if (event.key === 'Enter') {
            event.preventDefault();
            send(inputField.value);
            inputField.value = '';
        } else if (event.ctrlKey && event.key === 'c') {
            event.preventDefault();
            send(new Uint8Array([3]));
        } else if (event.ctrlKey && event.key === 'd') {
            event.preventDefault();
            send(new Uint8Array([4]));
        }
    });
    
    async function disconnect() {
        device_connected = false;
    
        if (globalReader) {
            try {
                await globalReader.cancel();
            } catch (error) {
                console.error('Error canceling the reader:', error);
            }
        }
    
        await readLoopExitSignal;
    
        readLoopExitSignal = new Promise(resolve => {
            readLoopExitSignalResolver = resolve;
        });
    
        try {
            await port.close();
            console.log('Port closed.');
        } catch (error) {
            console.error('Port close error:', error);
        }
    
        port = null;
        globalReader = null;
        updateStatus("Disconnected.");
        toggleConnectDisconnectUI(false);
    }
    
    function toggleConnectDisconnectUI(isConnected) {
        connectButton.style.display = isConnected ? 'none' : 'inline-block';
        disconnectButton.style.display = isConnected ? 'inline-block' : 'none';
        connectButton.disabled = isConnected;
        disconnectButton.disabled = !isConnected;
    }
    
    connectButton.addEventListener('click', () => {
        connect().catch(console.error);
    });
    
    disconnectButton.addEventListener('click', () => {
        disconnect().catch(console.error);
    });
    sendButton.addEventListener('click', () => {
        send(inputField.value).catch(error => {
            console.error('Send error:', error);
            updateStatus(`Send error: ${error}`);
        });
        inputField.value = '';
    });
    updateStatus("Ready to connect. Please select a baud rate and end of line character set and press 'Connect'.");
});