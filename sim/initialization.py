from .clock import clock


class NotInitializedError(RuntimeError):
    '''
    Report a missing or failed simulation initializer.

    Attributes:
        args (tuple): Exception arguments containing the error message.
    '''

    def __init__(
        self,
        message=(
            'Simulation is not initialized. '
            'Define init() and call sim.initialize(init) '
            'or sim.run(init, main) before using robot APIs.'
        ),
    ) -> None:
        super().__init__(message)


def require_initialized():
    '''
    Check that the program entered its own init function successfully.
    '''
    if clock._initialization is None:
        raise NotInitializedError()


def requires_init(function):
    '''
    Guard a robot operation until the program enters init.
    '''
    def checked(*args, **kwargs):
        require_initialized()
        return function(*args, **kwargs)
    return checked


def initialize(init=None, *args, **kwargs):
    '''
    Call the program initializer without imposing any device configuration.

    Arg:
        init (callable): User function named init.
        args (tuple): Positional arguments passed to init.
        kwargs (dict): Keyword arguments passed to init.

    Returns:
        object: Value returned by init.
    '''
    clock._initialization = None
    if not callable(init) or getattr(init, '__name__', None) != 'init':
        raise NotInitializedError()
    # Initialization may itself use wait, read an IMU, or configure devices.
    clock._initialization = 'initializing'
    try:
        result = init(*args, **kwargs)
    # If any exception happens, clear the init status and then raise error
    except BaseException:
        clock._initialization = None
        raise
    clock._initialization = True
    return result


def run(init=None, main=None) -> None:
    '''
    Run the program initializer before its main function.

    Arg:
        init (callable): User function named init.
        main (callable): Program body called after initialization.

    Returns:
        object: Value returned by main.
    '''
    if not callable(init) or getattr(init, '__name__', None) != 'init':
        clock._initialization = None
        raise NotInitializedError()
    if not callable(main):
        raise TypeError('main must be callable')
    initialize(init)
    return main()
