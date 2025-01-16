@app.route("/get_active_program", methods=["GET"])
def get_active_program():
    structure = get_structure('structure.json')
    return {"structure": structure}


def insert_code(file_path, dynamic_code):
    try:
        code_with_indent = textwrap.indent(dynamic_code, '  ')
        code = f"def program_main(should_terminate_function, set_active_block_id, publish_grip_state, orm_blockly_delay, orm_blockly_set_position, orm_blockly_set_gripper_state):\n{code_with_indent}"

        with open(file_path, 'w') as file:
            file.write(code)

    except Exception as e:
        print(f"Error: {e}")


def save_structure(file_path, structure):
    with open(file_path, 'w') as file:
        file.write(structure)


def get_structure(file_path):
    structure = None
    with open(file_path, 'r') as file:
        structure = file.read(structure)

    return structure

@app.route("/set_active_program", methods=["POST"])
def set_active_program():
    data = request.json
    source = data["source"]
    structure = data["structure"]

    insert_code('program.py', source)
    save_structure('structure.json', structure)
    return {"success": True}


@app.route("/get_program_state", methods=["GET"])
def get_program_state():
    global active_block_id
    return {"id": active_block_id}


@app.route('/get_blockly_state', methods=["GET"])
def get_blockly_state():
    global blockly_running
    return {"state": blockly_running}