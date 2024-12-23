import sqlite3
from contextlib import closing

# FIXME: if instance_config = True it can't access to needful db in the instance directory
from api.config import DB_PATH


class Db:

    def __init__(self):
        self.db_path = DB_PATH

    def execute(self, query, params=None, is_many=False):
        with closing(sqlite3.connect(self.db_path)) as conn:
            try:
                with conn:
                    with closing(conn.cursor()) as cursor:  # auto-closes
                        if is_many:
                            cursor.executemany(query, params or [])
                        else:
                            cursor.execute(query, params or [])

                        conn.commit()

                        if (cursor.description):
                            response = cursor.fetchall()
                        else:
                            response = cursor.lastrowid

                        return response
            except Exception as e:
                print('DB Exception', e)
                pass
