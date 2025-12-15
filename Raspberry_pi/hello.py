from flask import Flask, jsonify, abort, send_from_directory, render_template
import serial
import time
import sys

app = Flask(__name__)

# --------------------
# SERIAL COMMUNICATION
# --------------------

def open_serial(port="/dev/ttyS0", baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=1, write_timeout=1)
        print(f"[OK] Port série ouvert : {port} @ {baudrate}")
        return ser
    except Exception as e:
        print(f"[ERREUR] Port série : {e}")
        sys.exit(1)


def send_command(cmd):
    ser.write((cmd + "\n").encode())
    time.sleep(0.05)
    response = ser.read_all().decode(errors="ignore").strip()
    print(f"→ {cmd} → {response}")
    return response


# Ouverture du port série UNE SEULE FOIS
ser = open_serial("/dev/ttyS0", 115200)


# --------------------
# HTML
# --------------------

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

# --------------------
# DATA STORAGE
# --------------------

temperatures = []
pressures = []
scale = 1.0


# --------------------
# CREATE
# --------------------

@app.route('/temp/', methods=['POST'])
def create_temperature():
    response = send_command("GET_T")

    try:
        value = float(response)
    except ValueError:
        return jsonify({"error": "Invalid temperature from STM32"}), 500

    temperatures.append(value)
    return jsonify({
        "message": "Temperature retrieved",
        "index": len(temperatures) - 1,
        "value": value
    }), 201


@app.route('/pres/', methods=['POST'])
def create_pressure():
    response = send_command("GET_P")

    try:
        value = float(response)
    except ValueError:
        return jsonify({"error": "Invalid pressure from STM32"}), 500

    pressures.append(value)
    return jsonify({
        "message": "Pressure retrieved",
        "index": len(pressures) - 1,
        "value": value
    }), 201


# --------------------
# RETRIEVE
# --------------------

@app.route('/temp/', methods=['GET'])
def get_all_temperatures():
    return jsonify(temperatures)


@app.route('/temp/<int:x>', methods=['GET'])
def get_temperature(x):
    if x < 0 or x >= len(temperatures):
        abort(404)
    return jsonify({"index": x, "value": temperatures[x]})


@app.route('/pres/', methods=['GET'])
def get_all_pressures():
    return jsonify(pressures)


@app.route('/pres/<int:x>', methods=['GET'])
def get_pressure(x):
    if x < 0 or x >= len(pressures):
        abort(404)
    return jsonify({"index": x, "value": pressures[x]})


@app.route('/scale/', methods=['GET'])
def get_scale():
    return jsonify({"scale": scale})


@app.route('/angle/', methods=['GET'])
def get_angle():
    if not temperatures:
        return jsonify({"error": "No temperature available"}), 400

    angle = temperatures[-1] * scale
    return jsonify({
        "temperature": temperatures[-1],
        "scale": scale,
        "angle": angle
    })


# --------------------
# UPDATE
# --------------------

@app.route('/scale/<int:x>', methods=['POST'])
def update_scale(x):
    global scale
    data = app.current_request.get_json(silent=True)

    if not data or 'value' not in data:
        return jsonify({"error": "Missing 'value'"}), 400

    scale = data['value']
    return jsonify({
        "message": f"Scale updated for {x}",
        "scale": scale
    })


# --------------------
# DELETE
# --------------------

@app.route('/temp/<int:x>', methods=['DELETE'])
def delete_temperature(x):
    if x < 0 or x >= len(temperatures):
        abort(404)

    return jsonify({
        "message": "Temperature deleted",
        "value": temperatures.pop(x)
    })


@app.route('/pres/<int:x>', methods=['DELETE'])
def delete_pressure(x):
    if x < 0 or x >= len(pressures):
        abort(404)

    return jsonify({
        "message": "Pressure deleted",
        "value": pressures.pop(x)
    })


# --------------------
# ERRORS
# --------------------

@app.errorhandler(404)
def not_found(error):
    return jsonify({"error": "Not found"}), 404


# --------------------
# MAIN
# --------------------

if __name__ == '__main__':
    try:
        app.run(debug=True)
    finally:
        ser.close()
        print("[OK] Port série fermé")

