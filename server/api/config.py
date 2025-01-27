import os

DEBUG = False
HOST = "0.0.0.0"
PORT = 5001
SECRET_KEY = ""

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))

DB_PATH = os.path.join(BASE_DIR, "instance", "orm.db")
SCHEMA_PATH = os.path.join(BASE_DIR, "migrations", "schema.sql")
