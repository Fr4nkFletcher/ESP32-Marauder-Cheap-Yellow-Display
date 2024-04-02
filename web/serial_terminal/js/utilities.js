/**
 * @name toByteArray
 * Convert a string to a byte array
 */
function toByteArray(str) {
  let byteArray = [];
  for (let i = 0; i < str.length; i++) {
    let charcode = str.charCodeAt(i);
    if (charcode <= 0xFF) {
      byteArray.push(charcode);
    }
  }
  return byteArray;
}

function fromByteArray(byteArray) {
  return String.fromCharCode.apply(String, byteArray);
}

function crc32(data, value=0) {
  if (data instanceof Array) {
    data = fromByteArray(data);
  }
  let table = [];
  for(let entry, c = 0; c < 256; c++) {
    entry = c;
    for(let k = 0; k < 8; k++) {
      entry = 1 & entry ? 3988292384^entry >>> 1 : entry >>> 1;
    }
    table[c] = entry;
  }
  let n = -1 - value;
  for(let t = 0; t < data.length; t++) {
    n = n >>> 8^table[255 & (n^data.charCodeAt(t))];
  }
  return (-1 ^ n) >>> 0;
}

function zipLongest() {
    var args = [].slice.call(arguments);
    var longest = args.reduce(function(a,b){
        return a.length > b.length ? a : b
    }, []);

    return longest.map(function(_,i){
        return args.map(function(array){return array[i]})
    });
}

class struct {
    static lut = {
      "b": {u: DataView.prototype.getInt8, p: DataView.prototype.setInt8, bytes: 1},
      "B": {u: DataView.prototype.getUint8, p: DataView.prototype.setUint8, bytes: 1},
      "h": {u: DataView.prototype.getInt16, p: DataView.prototype.setInt16, bytes: 2},
      "H": {u: DataView.prototype.getUint16, p: DataView.prototype.setUint16, bytes: 2},
      "i": {u: DataView.prototype.getInt32, p: DataView.prototype.setInt32, bytes: 4},
      "I": {u: DataView.prototype.getUint32, p: DataView.prototype.setUint32, bytes: 4},
      "q": {u: DataView.prototype.getInt64, p: DataView.prototype.setInt64, bytes: 8},
      "Q": {u: DataView.prototype.getUint64, p: DataView.prototype.setUint64, bytes: 8},
    }

    static pack(...args) {
        let format = args[0];
        let pointer = 0;
        let data = args.slice(1);
        if (format.replace(/[<>]/, '').length != data.length) {
            throw("Pack format to Argument count mismatch");
            return;
        }
        let bytes = [];
        let littleEndian = true;
        for (let i = 0; i < format.length; i++) {
            if (format[i] == "<") {
                littleEndian = true;
            } else if (format[i] == ">") {
                littleEndian = false;
            } else {
                pushBytes(format[i], data[pointer]);
                pointer++;
            }
        }

        function pushBytes(formatChar, value) {
            if (!(formatChar in struct.lut)) {
                throw("Unhandled character '" + formatChar + "' in pack format");
            }
            let dataSize = struct.lut[formatChar].bytes;
            let view = new DataView(new ArrayBuffer(dataSize));
            let dataViewFn = struct.lut[formatChar].p.bind(view);
            dataViewFn(0, value, littleEndian);
            for (let i = 0; i < dataSize; i++) {
                bytes.push(view.getUint8(i));
            }
        }

        return bytes;
    };

    static unpack(format, bytes) {
        let pointer = 0;
        let data = [];
        let littleEndian = true;

        for (let c of format) {
            if (c == "<") {
                littleEndian = true;
            } else if (c == ">") {
                littleEndian = false;
            } else {
                pushData(c);
            }
        }

        function pushData(formatChar) {
            if (!(formatChar in struct.lut)) {
                throw("Unhandled character '" + formatChar + "' in unpack format");
            }
            let dataSize = struct.lut[formatChar].bytes;
            let view = new DataView(new ArrayBuffer(dataSize));
            for (let i = 0; i < dataSize; i++) {
              view.setUint8(i, bytes[pointer + i] & 0xFF);
            }
            let dataViewFn = struct.lut[formatChar].u.bind(view);
            data.push(dataViewFn(0, littleEndian));
            pointer += dataSize;
        }

        return data;
    };

    static calcsize(format) {
        let size = 0;
        for (let i = 0; i < format.length; i++) {
            if (format[i] != "<" && format[i] != ">") {
                size += struct.lut[format[i]].bytes;
            }
        }

        return size;
    }
}

function* makeFileIterator(content) {
    for (let line of content.split(/\r?\n/)) {
        yield line.trim();
    }
    return '';
}
