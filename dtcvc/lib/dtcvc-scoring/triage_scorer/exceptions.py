class RequiredFieldNotSetException(Exception):
    def __init__(self, field):
        super().__init__(f"Required field '{field}' not set.")


class FieldInvalidException(Exception):
    def __init__(self, field, value, value_format=None):
        message = f"Field '{field}' with value '{value}' invalid."
        if value_format != None:
            message += f" '{field}' must follow format {value_format}"
        super().__init__(message)


class UnknownFieldException(Exception):
    def __init__(self, field):
        super().__init__(f"Input field '{field}' not recognized and is invalid.")
