class ValidationError(Exception):
    """Raised when a request fails validation; carries a list of messages."""

    def __init__(self, errors):
        super().__init__("; ".join(errors))
        self.errors = list(errors)
