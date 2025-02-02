import os

from datetime import datetime


def test_start_program(client):
    """Test the POST /blockly/start route."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    test_data_dir = os.path.join(base_dir, "test_data")
    program_file_path = os.path.join(test_data_dir, "program.txt")
    structure_file_path = os.path.join(test_data_dir, "structure.json")

    with open(program_file_path, "r") as program_file:
        program = program_file.read()

    with open(structure_file_path, "r") as structure_file:
        structure = structure_file.read()

    data = {"program": program, "structure": structure}

    response = client.post("/blockly/start", json=data)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 200
    assert response_data["status"] == "success"
    assert response_data["message"] == "Program started successfully."


def test_start_program_when_already_running(client):
    """Test the POST /blockly/start route with already running program"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    test_data_dir = os.path.join(base_dir, "test_data")
    program_file_path = os.path.join(test_data_dir, "program.txt")
    structure_file_path = os.path.join(test_data_dir, "structure.json")

    with open(program_file_path, "r") as program_file:
        program = program_file.read()

    with open(structure_file_path, "r") as structure_file:
        structure = structure_file.read()

    data = {"program": program, "structure": structure}

    response = client.post("/blockly/start", json=data)
    response_data = response.json

    response = client.post("/blockly/start", json=data)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert response_data["message"] == "Program is already running."


def test_start_program_without_program(client):
    """Test the POST /blockly/start route without any program"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    test_data_dir = os.path.join(base_dir, "test_data")
    structure_file_path = os.path.join(test_data_dir, "structure.json")

    with open(structure_file_path, "r") as structure_file:
        structure = structure_file.read()

    data = {"structure": structure}
    response = client.post("/blockly/start", json=data)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program' and 'structure' must be provided in the request"
    )
    assert response_data["data"] is None


def test_start_program_without_structure(client):
    """Test the POST /blockly/start route without any structure"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    test_data_dir = os.path.join(base_dir, "test_data")
    program_file_path = os.path.join(test_data_dir, "program.txt")

    with open(program_file_path, "r") as program_file:
        program = program_file.read()

    data = {"program": program}
    response = client.post("/blockly/start", json=data)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program' and 'structure' must be provided in the request"
    )
    assert response_data["data"] is None


def test_start_program_without_params(client):
    """Test the POST /blockly/start route without program and structure"""
    response = client.post("/blockly/start", json={})
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program' and 'structure' must be provided in the request"
    )
    assert response_data["data"] is None


def test_get_program_with_invalid_id(client):
    """Test the GET /blockly/<int:id> route with invalid id"""
    response = client.get("/blockly/99999")
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert response_data["message"] == "No program found with the provided ID"
    assert response_data["data"] is None


def test_stop_program(client):
    """Test the POST /blockly/stop route"""
    response = client.post("/blockly/stop")
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 200
    assert response_data["status"] == "success"


def test_delete_program_with_nonexistent_id(client):
    """Test the DELETE /blockly route with nonexistent id"""
    unique_id = 99999
    query_params = {"id": unique_id}

    delete_response = client.delete("/blockly", query_string=query_params)
    response_data = delete_response.json

    print("response_data -> ", response_data)

    assert delete_response.status_code == 200

    get_response = client.get(f"/blockly/{unique_id}")
    response_data = get_response.json

    print("response_data -> ", response_data)

    assert get_response.status_code == 400
    assert response_data["status"] == "error"
    assert response_data["message"] == "No program found with the provided ID"


def test_get_programs(client):
    """Test the GET /blockly route"""
    response = client.get("/blockly")
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 200


def test_save_program(client):
    """Test the POST /blockly route"""
    body = {
        "program_name": "Test program name" + str(datetime.now()),
        "program_description": "Test program description",
        "program_structure": "Test program structure",
    }

    response = client.post("/blockly", json=body)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 201
    assert response_data["status"] == "success"


def test_save_program_without_body_params(client):
    """Test the POST /blockly route without body params"""
    body = {
        "program_description": "Test program description",
        "program_structure": "Test program structure",
    }

    response = client.post("/blockly", json=body)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program_name', 'program_description' and 'program_structure' must be provided in the request"
    )

    body.pop("program_description")

    response = client.post("/blockly", json=body)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program_name', 'program_description' and 'program_structure' must be provided in the request"
    )

    body.pop("program_structure")

    response = client.post("/blockly", json=body)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program_name', 'program_description' and 'program_structure' must be provided in the request"
    )


def test_update_program(client):
    """Test the PATCH /blockly route"""
    query_params = {"id": "0"}

    body = {
        "program_name": "Updated program_name" + str(datetime.now()),
        "program_description": "Updated program_description",
        "program_structure": "Updated program_structure",
    }

    response = client.patch("/blockly", json=body, query_string=query_params)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 200


def test_update_program_without_id(client):
    """Test the PATCH /blockly route"""
    body = {
        "program_name": "Updated program_name" + str(datetime.now()),
        "program_description": "Updated program_description",
        "program_structure": "Updated program_structure",
    }

    response = client.patch("/blockly", json=body)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "The program id should be provided as a query parameter - /blockly?id=0"
    )


def test_update_program_without_name(client):
    """Test the PATCH /blockly route without name"""
    query_params = {"id": "0"}

    body = {
        "program_description": "Updated program_description",
        "program_structure": "Updated program_structure",
    }

    response = client.patch("/blockly", json=body, query_string=query_params)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program_name', 'program_description' and 'program_structure' must be provided in the request"
    )


def test_update_program_without_description(client):
    """Test the PATCH /blockly route without description"""
    query_params = {"id": "0"}

    body = {
        "program_name": "Updated program_name" + str(datetime.now()),
        "program_structure": "Updated program_structure",
    }

    response = client.patch("/blockly", json=body, query_string=query_params)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program_name', 'program_description' and 'program_structure' must be provided in the request"
    )


def test_update_program_without_structure(client):
    """Test the PATCH /blockly route without structure"""
    query_params = {"id": "0"}

    body = {
        "program_name": "Updated program_name" + str(datetime.now()),
        "program_description": "Updated program_description",
    }

    response = client.patch("/blockly", json=body, query_string=query_params)
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 400
    assert response_data["status"] == "error"
    assert (
        response_data["message"]
        == "Both 'program_name', 'program_description' and 'program_structure' must be provided in the request"
    )


def test_get_current_program(client):
    """Test the GET /blockly/current-program route"""
    response = client.get("blockly/current-program")
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 200
    assert response_data["status"] == "success"
    assert response_data["data"] != {}


def test_get_current_block_id(client):
    """Test the GET /blockly/current-block-id route"""
    response = client.get("blockly/current-block-id")
    response_data = response.json

    print("response_data -> ", response_data)

    assert response.status_code == 200
    assert response_data["status"] == "success"
    assert response_data["data"] != {}
