from datetime import datetime

def test_start_program(client):
    """Test the /blockly/start route."""
    data = {"id": "1"}
    response = client.post('/blockly/start', json=data)
    assert response.status_code == 200
    assert b"Start" in response.data

def test_stop_program(client):
    """Test the home page route."""
    response = client.post('/blockly/stop')
    assert response.status_code == 200
    assert b"Stop" in response.data

def test_delete_program(client):
    query_params = {"id": "11"}
    response = client.delete('/blockly', query_string=query_params)
    assert response.status_code == 200

def test_get_programs(client):
    response = client.get('/blockly')
    assert response.status_code == 200

def test_save_program(client):
    body = {
        'program_name': 'Test program_name' + str(datetime.now()),
        'program_description': 'Test program_description',
        'program_structure': 'Test program_structure',
        'is_running': '0',
    }

    response = client.post('/blockly', json=body)
    assert response.status_code == 200

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