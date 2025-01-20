from datetime import datetime

def test_start_program(client):
    """Test the /blockly/start route."""
    data = {"id": "5"}
    response = client.post('/blockly/start', json=data)
    assert response.status_code == 200

    response_data = response.json
    assert response_data['status'] == 'success'

def test_start_program_without_id(client):
    response = client.post('/blockly/start', json={})
    assert response.status_code == 400

    response_data = response.json
    
    assert response_data['status'] == 'error'
    assert response_data['message'] == "The program ID was not provided in the request"
    assert response_data['data'] is None

def test_start_program_with_invalid_id(client):
    response = client.post('/blockly/start', json={"id": 9999})
    assert response.status_code == 400

    response_data = response.json
    
    assert response_data['status'] == 'error'
    assert response_data['message'] == "No program found with the provided ID"
    assert response_data['data'] is None

def test_stop_program(client):
    """Test the home page route."""
    response = client.post('/blockly/stop')
    assert response.status_code == 200
    assert b"Stop" in response.data

def test_delete_program(client):
    # Add the item
    body = {
        'program_name': 'Test program name' + str(datetime.now()),
        'program_description': 'Test program description',
        'program_structure': 'Test program structure',
        'is_running': '0',
    }
    add_response = client.post('/blockly', json=body)
    assert add_response.status_code == 201

    # FIXME: unique_id should be gotten from add_response
    unique_id = 11
    query_params = {"id": unique_id}

    # Delete the item
    delete_response = client.delete('/blockly', query_string=query_params)
    assert delete_response.status_code == 200

    # Verify the item no longer exists
    # TODO: get the item by id

def test_get_programs(client):
    response = client.get('/blockly')
    assert response.status_code == 200

def test_save_program(client):
    body = {
        'program_name': 'Test program name' + str(datetime.now()),
        'program_description': 'Test program description',
        'program_structure': 'Test program structure',
        'is_running': '0',
    }

    response = client.post('/blockly', json=body)
    assert response.status_code == 201

    response_data = response.json
    assert response_data['status'] == 'success'

def test_update_program(client):
    query_params = {"id": "14"}

    body = {
        'program_name': 'Updated program_name' + str(datetime.now()),
        'program_description': 'Updated program_description',
        'program_structure': 'Updated program_structure',
        'is_running': '0',
    }

    response = client.patch('/blockly', json=body, query_string=query_params)
    assert response.status_code == 200