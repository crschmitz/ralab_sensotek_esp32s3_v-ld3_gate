#!/usr/bin/env python3
import argparse
import json
import sys
import time
import zlib

try:
    import serial  # pyserial
except ImportError:
    print("Error: pyserial not installed. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def normalize_port(p: str) -> str:
    # Accept "\dev\ttyACM0" and normalize to "/dev/ttyACM0"
    if p.startswith("\\dev\\"):
        return p.replace("\\", "/")
    return p


def build_cmd_message(msg_id: int, cmd: str) -> str:
    obj = {"id": msg_id, "cmd": cmd}
    # compact JSON, CRLF terminated
    return json.dumps(obj, separators=(",", ":")) + "\r\n"


def build_cfg_message(msg_id: int, filename: str) -> str:
    # Read file as BYTES to preserve original line endings (\r\n or \n)
    try:
        with open(filename, "rb") as f:
            raw = f.read()
    except Exception as e:
        print(f"Error reading file '{filename}': {e}", file=sys.stderr)
        sys.exit(1)

    # Decode to text as receiver will see it in the 'cfg' field
    # (UTF-8 is typical; errors='replace' ensures we can always serialize)
    cfg_text = raw.decode("utf-8", errors="replace")

    # print("Canonical:", cfg_text)

    # CRC32 over the EXACT text that will be in 'cfg' after JSON decoding on receiver
    crc_value = zlib.crc32(cfg_text.encode("utf-8")) & 0xFFFFFFFF

    obj = {
        "id": msg_id,
        "cmd": "cfg",
        "file": cfg_text,   # newlines preserved; json.dumps will escape them as \n or \r\n
        "crc": crc_value
    }
    # ensure_ascii=False to keep unicode readable on the wire (optional)
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\r\n"

def read_json_string(ser, timeout=1.0):
    """
    Reads a complete JSON message from a serial port.
    Waits for the first '{' and stops when all braces are balanced.
    Returns the raw JSON string (including braces) or None if timeout/no valid data.
    """
    import time
    start_time = time.time()

    buf = ""
    brace_count = 0
    in_json = False

    while True:
        if time.time() - start_time > timeout:
            if not in_json:
                print("No response (timeout)")
            else:
                print("Incomplete JSON (timeout)")
            return None

        c = ser.read(1).decode("utf-8", errors="replace")
        
        if not c:
            continue  # no new byte, keep waiting

        if c == '{':
            brace_count += 1
            in_json = True
        elif c == '}':
            brace_count -= 1

        if in_json:
            buf += c

        if in_json and brace_count == 0:
            break

    # Return the raw JSON string (unparsed)
    return buf


def crc32_field(field):
    # If it's a dict (from parsed JSON), serialize it in canonical form
    # if isinstance(field, dict):
    #     payload = json.dumps(field, separators=(",", ":"), ensure_ascii=False)
    # else:
    #     # Assume it's already a string (for example, extracted from JSON)
    #     payload = str(field)

    payload = str(field)

    # print("Canonical:", payload)

    crc = zlib.crc32(payload.encode("utf-8")) & 0xFFFFFFFF
    return crc, payload


def extract_res_field(raw_json: str):
    start = raw_json.find('"res"')
    if start == -1:
        return None

    # find the first '{' after "res"
    start = raw_json.find('{', start)
    if start == -1:
        return None

    brace = 1
    i = start + 1
    while i < len(raw_json) and brace > 0:
        if raw_json[i] == '{':
            brace += 1
        elif raw_json[i] == '}':
            brace -= 1
        i += 1

    return raw_json[start:i] if brace == 0 else None

def main():
    parser = argparse.ArgumentParser(description="Send JSON command/cfg over a serial port.")
    parser.add_argument("--id", type=int, default=1, help="Message ID (default: 1)")
    parser.add_argument("--cmd", default="get", help='Command string when not sending a file (default: "get")')
    parser.add_argument("-f", "--filename", help="If provided, send a cfg message with file contents + crc")
    parser.add_argument("--port", default="/dev/ttyACM0", help=r'Serial port (default: "/dev/ttyACM0"; "\\dev\\ttyACM0" also accepted)')
    parser.add_argument("--time", type=float, default=0, help="Time interval between repeated commands (default: 0 = no repetition)")
    parser.add_argument('--debug', action='store_true', default=False, help='Activate debug mode')
    args = parser.parse_args()

    port = normalize_port(args.port)

    if args.filename:
        line = build_cfg_message(args.id, args.filename)
    else:
        line = build_cmd_message(args.id, args.cmd)

    print("Sending: ", line.strip())
    print()

    with serial.Serial(port, baudrate=921600, timeout=0.5) as ser:           

        while True:
            try:       
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(line.encode("utf-8"))
                ser.flush()
                response = read_json_string(ser, timeout=0.25)
                if response is None:
                    print("No valid JSON received")
                    continue
                else:
                    print("Received:", response)

                    msg = json.loads(response)
                    if args.debug:
                        for key, value in msg.items():
                            print(f"  {key}: {value}")

                        if msg.get("cmd") == "get":
                            if "crc" in msg:
                                res = extract_res_field(response)
                                crc_value = zlib.crc32(res.encode("utf-8")) & 0xFFFFFFFF
                                if crc_value == msg.get("crc"):
                                    print("CRC OK!")
                                else:
                                    print(f"CRC MISMATCH! Calculated: {crc_value}, Expected: {msg.get('crc')}")
                if args.time > 0:
                    time.sleep(args.time)
                else:
                    break
    
            except Exception as e:
                print(f"Error opening/sending to serial port '{port}': {e}", file=sys.stderr)
                pass


if __name__ == "__main__":
    main()
