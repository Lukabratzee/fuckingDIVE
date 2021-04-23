from flask import Flask, render_template, request, jsonify

app = Flask(__name__)


@app.route("/")
def home():
    return render_template('index.html')


@app.route("/data", methods=['POST'])
def data():
    token = request.headers.get('X-Api-Key')
    print(token)
    content = request.get_json()
    if 'X-Api-Key' in request.headers:
        if token == '123':
            if 'value' in content:
                print(content)
                return jsonify({"message": "OK: Authorized"}), 200
        else:
            return jsonify({'message': 'a valid token is missing'}), 404


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
