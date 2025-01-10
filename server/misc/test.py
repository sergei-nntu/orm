@app.route("/start_program", methods=["GET"])
def start_program():
    global program_thread
    global should_terminate_flag
    global blockly_running

    blockly_running = True
    should_terminate_flag = False

    if program_thread is None or not program_thread.is_alive():
        imp.reload(program)
        program_thread = threading.Thread(target=program.program_main, args=(
            should_terminate, set_active_block, publish_grip_state, orm_blockly_delay, orm_blockly_set_position,
            orm_blockly_set_gripper_state))
        program_thread.start()
        return {"success": True}
    else:
        return {"success": False}

@app.route("/stop_program", methods=["GET"])
def stop_program():
    global should_terminate_flag, blockly_running
    if should_terminate_flag:
        return {"success": False}
    else:
        blockly_running = False
        should_terminate_flag = True
        return {"success": True}


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
