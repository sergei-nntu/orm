def test_blockly(client):
    """Test the /blockly/start route."""
    data = {"id": "1"}
    response = client.post('/blockly/start', json=data)
    assert response.status_code == 200
    assert b"Start" in response.data