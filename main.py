"""`main` is the top level module for your Flask application."""

# Import the Flask Framework
from flask import Flask, request, render_template
from google.appengine.ext import ndb
app = Flask(__name__)
# Note: We don't need to call run() since our application is embedded within
# the App Engine WSGI application server.

class Data(ndb.Model):
   datastringtime = ndb.DateTimeProperty(auto_now_add=True)
   temperature = ndb.FloatProperty()
   relativehumidity = ndb.FloatProperty()
   
@app.route('/')   
   
def home():
   query = Data.query().order(Data.datastringtime)
   return render_template('home.html',results=query)
   
@app.route('/log')

def log():
   datastringtemp = request.args.get('T','9999')
   datastringhum = request.args.get('rH','9999')
   data = Data(temperature = float(datastringtemp), relativehumidity = float(datastringhum))
   data.put()
   # """Return a friendly HTTP greeting."""
   return 'DATA DATA'+str(data)


@app.errorhandler(404)
def page_not_found(e):
   """Return a custom 404 error."""
   return 'Sorry, Nothing at this URL.', 404
