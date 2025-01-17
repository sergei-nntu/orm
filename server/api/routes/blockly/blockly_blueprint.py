from flask import Blueprint

from .get_programs import get_programs
from .save_program import save_program
from .stop_program import stop_program
from .start_program import start_program
from .update_program import update_program
from .delete_program import delete_program
from .set_active_program import set_active_program
from .get_active_program import get_active_program

blockly = Blueprint('blockly', __name__)

blockly.add_url_rule('/blockly', view_func=get_programs, methods=['GET'])
blockly.add_url_rule('/blockly', view_func=save_program, methods=['POST'])
blockly.add_url_rule('/blockly', view_func=update_program, methods=['PATCH'])
blockly.add_url_rule('/blockly', view_func=delete_program, methods=['DELETE'])

blockly.add_url_rule('/blockly/stop', view_func=stop_program, methods=['POST']) 
blockly.add_url_rule('/blockly/start', view_func=start_program, methods=['POST'])

blockly.add_url_rule('/blockly/set_active_program', view_func=set_active_program, methods=['POST'])
blockly.add_url_rule('/blockly/get_active_program', view_func=get_active_program, methods=['GET'])
