def register_blueprints(app):
    from .blockly import blockly

    app.register_blueprint(blockly, url_prefix='/blockly')