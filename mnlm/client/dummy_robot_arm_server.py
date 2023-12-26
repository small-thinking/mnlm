from flask import Flask, jsonify, request

app = Flask(__name__)


@app.route("/execute", methods=["POST"])
def execute_operations():
    operations = request.json.get("operations", [])
    # Process the operations here
    processed_operations = [
        {"operation": op["operation"], "status": "processed"} for op in operations
    ]
    print(f"Process {processed_operations}")
    return jsonify(processed_operations)


if __name__ == "__main__":
    app.run(debug=True, port=5000)
