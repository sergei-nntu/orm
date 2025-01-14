from flask import jsonify

from dao import BlocklyDAO


def get_programs():
    try:
        blockly_dao = BlocklyDAO()
        programs = blockly_dao.get_programs()
        return jsonify({'programs': programs}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500