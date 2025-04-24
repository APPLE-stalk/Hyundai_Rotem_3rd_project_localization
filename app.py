from server.flask_server import app as flask_app
from server.dash_server import create_dash_app
from server.thread_manager import run_multithread


from utils.config import SHARED



def run_flask():
    flask_app.run(port=5050, debug=False, use_reloader=False)

def run_dash():
    create_dash_app().run(port=8050, debug=False, use_reloader=False)

if __name__ == '__main__':

    run_multithread(run_flask, run_dash)
