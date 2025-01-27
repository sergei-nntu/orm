import pytest
from api import create_app


# NOTE: use command `pytest -v` in cli to run all tests
@pytest.fixture
def app():
    # NOTE: Use test config by adding a parameter in the create_app() function
    # test_config = {
    #     'TESTING': True,
    #     'DATABASE': db_path,
    # }

    app = create_app()

    yield app


@pytest.fixture
def client(app):
    return app.test_client()


@pytest.fixture
def runner(app):
    return app.test_cli_runner()
