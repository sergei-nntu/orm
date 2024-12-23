from flask import Blueprint

blockly = Blueprint('blockly', __name__)


@blockly.route('/test', methods=('GET'))
def test():
    return 'This is a test route.'

# TODO: release these routes
# @blockly.route("/programs", methods=["GET"])
# @blockly.route("/start", methods=["POST"])
# @blockly.route("/stop", methods=["POST"])

