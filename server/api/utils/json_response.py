from flask import jsonify
from typing import Any, Optional, Dict, Union

class JsonResponse:
    def __init__(self, status: str, data: Optional[Dict[str, Any]] = None, message: str = "", error: Optional[Union[str, Dict[str, Any]]] = None):
        self.response = {
            "status": status,
            "data": data,
            "message": message,
            "error": error
        }

    def to_json(self, status_code: int = 200):
        return jsonify(self.response), status_code