from flask import Flask

flask_app = Flask(__name__)

from gauss_rpi.wifi import flask_views
