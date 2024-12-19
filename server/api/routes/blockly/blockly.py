from flask import Blueprint

blockly = Blueprint('blockly', __name__)

@blockly.route('/test', methods=('GET'))
    return 'This is a test route.'