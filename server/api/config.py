import os

DEBUG = True
HOST = '0.0.0.0'
PORT = 5000
SECRET_KEY = ''

BASE_DIR = os.path.abspath(os.path.dirname(__file__))
DB_PATH = os.path.join(BASE_DIR, 'instance', 'orm.db')