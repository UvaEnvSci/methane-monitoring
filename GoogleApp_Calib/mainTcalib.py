"""`main` is the top level module for your Flask application."""

# Import the Flask Framework
from flask import Flask, request, render_template
from google.appengine.ext import ndb
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
   query1 = Data.query(Data.ID == '1').order(Data.datastringtime)
   query2 = Data.query(Data.ID == '2').order(Data.datastringtime)
   query3 = Data.query(Data.ID == '3').order(Data.datastringtime)
   query4 = Data.query(Data.ID == '4').order(Data.datastringtime)
   query5 = Data.query(Data.ID == '5').order(Data.datastringtime)
   query6 = Data.query(Data.ID == '6').order(Data.datastringtime)
   query7 = Data.query(Data.ID == '7').order(Data.datastringtime)
   query8 = Data.query(Data.ID == '8').order(Data.datastringtime)
   query9 = Data.query(Data.ID == '9').order(Data.datastringtime)
   query10 = Data.query(Data.ID == '10').order(Data.datastringtime)
   query11 = Data.query(Data.ID == '11').order(Data.datastringtime)
   return render_template('home.html',results1=query1,results2=query2,results3=query3,results4=query4,results5=query5,results6=query6,results7=query7,results8=query8,results9=query9,results10=query10,results11=query11)
   
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


@app.errorhandler(404)
def page_not_found(e):
   """Return a custom 404 error."""
   return 'Sorry, Nothing at this URL.', 404
