def test_blockly(client):
    """Test the home page route."""
    response = client.post('/blockly/stop')
    assert response.status_code == 200
    assert b"Stop" in response.data