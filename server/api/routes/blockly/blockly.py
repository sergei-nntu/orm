from flask import Blueprint, request, jsonify, g


blockly = Blueprint('blockly', __name__)


@blockly.route('/programs', methods=['GET'])
def get_blockly_programs():
    query = "SELECT id, username FROM user"
    try:
        users = g.db.execute(query)
        return jsonify({'users': users}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@blockly.route('/start', methods=['POST'])
def start_blockly_program():
    data = request.get_json()
    username = data.get('username')
    password = data.get('password')
    
    if not username or not password:
        return jsonify({'error': 'Username and password are required'}), 400
    
    query = "INSERT INTO user (username, password) VALUES (?, ?)"
    try:
        user_id = g.db.execute(query, (username, password))
        return jsonify({'message': 'User added', 'user_id': user_id}), 201
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@blockly.route('/stop', methods=['POST'])
def stop_blockly_program():
    return 'Stop selected program'


@blockly.route('/save', methods=['POST'])
def save_blockly_program():
    return 'Save selected program'