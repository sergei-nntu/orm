import os

from flask import Flask, g
from dao import Db

from .utils import configure_logging


def create_app(test_config=None, instance_config=False):
    app = Flask(__name__, instance_relative_config=instance_config)
    app.config.from_mapping(
        # a default secret that should be overridden by instance config
        # SECRET_KEY="dev",
        # store the database in the instance folder
        # DATABASE=os.path.join(app.instance_path, "flaskr.sqlite"),
    )

    if test_config is None:
        # load the instance config, if it exists, when not testing
        app.config.from_pyfile("config.py", silent=True)
    else:
        # load the test config if passed in
        app.config.update(test_config)

    # TODO: define default params
    db_path = app.config.get("DB_PATH")
    schema_path = app.config.get("SCHEMA_PATH")

    # Initialize the Db instance and store it in app context
    app.db = Db(db_path, schema_path)

    # Initialize the database schema if it doesn't exist
    app.db.initialize_db()

    # Configure logging
    configure_logging()

    @app.before_request
    def before_request():
        g.db = app.db

    @app.teardown_request
    def teardown_request(exception):
        g.pop("db", None)

    # Register blueprints right here
    from .routes import register_blueprints

    register_blueprints(app)

    # ensure the instance folder exists
    try:
        os.makedirs(app.instance_path)
    except OSError:
        pass

    return app
