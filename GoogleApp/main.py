"""`main` is the top level module for your Flask application."""

# Import the Flask Framework
import calendar, datetime, json
from flask import Flask, request, render_template, Response
from google.appengine.ext import ndb
# from models import Entry
app = Flask(__name__)
# Note: We don't need to call run() since our application is embedded within
# the App Engine WSGI application server.


class Data(ndb.Model):
   datastringtime = ndb.DateTimeProperty(auto_now_add=True)
   ID = ndb.StringProperty()
   temperature = ndb.FloatProperty()
   relativehumidity = ndb.FloatProperty()
   gassensor = ndb.FloatProperty()
   pressure = ndb.FloatProperty()
   battery = ndb.FloatProperty()
   

@app.route('/delete')
def delete():
   entries = Data.query().fetch(keys_only=True)
   ndb.delete_multi(entries)
   # This could bulk delete 1000 entities a time
   return 'entries deleted'
      
   
@app.route('/')   
def home():
   query = Data.query().order(Data.datastringtime)
   return render_template('home.html',results=query)
   
@app.route('/log')
def log():
   datastringID = request.args.get('ID','NA')
   datastringtemp = request.args.get('T','9999')
   datastringhum = request.args.get('rH','9999')
   datastringgassensor = request.args.get('CH4','9999')
   datastringpressure = request.args.get('P','9999')
   datastringbattery = request.args.get('Batt','9999')
   data = Data(ID = datastringID, temperature = float(datastringtemp)/100.0, relativehumidity = float(datastringhum)/100.0, gassensor = float(datastringgassensor), pressure = float(datastringpressure), battery = float(datastringbattery)/100.0)
   data.put()
   return '0'

def handler(obj):
#   if hasattr(obj, 'isoformat'):
    if hasattr(obj, 'utctimetuple'):
#      return obj.isoformat()
       return calendar.timegm(obj.utctimetuple()) 
    else:
       raise TypeError, 'Object of type %s with value of %s is not JSON serializable' % (type(obj), repr(obj))
   
@app.route('/api')
def api():

   prevrequestdatetime = request.args.get('time',None)
   if prevrequestdatetime is None:
       query = Data.query().order(Data.datastringtime)
   else:
       dt = datetime.datetime.fromtimestamp(float(prevrequestdatetime))
       query = Data.query(Data.datastringtime > dt).order(Data.datastringtime)
   js = json.dumps([p.to_dict() for p in query.fetch()], default=handler)
   resp = Response(js, status=200, mimetype='application/json')
   return resp

   
@app.errorhandler(404)
def page_not_found(e):
   """Return a custom 404 error."""
   return 'Sorry, Nothing at this URL.', 404
