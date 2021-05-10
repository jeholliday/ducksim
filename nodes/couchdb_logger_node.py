#!/usr/bin/env python

import couchdb
import rospy
from rosgraph_msgs.msg import Log
import yaml

if __name__ == '__main__':
    # Create connection to CouchDB
    couch = couchdb.Server('http://admin:cloud@45.33.119.237:30006/')
    # Try to connect to database
    try:
        db = couch['rosout']
        print("Connected to database...")
    except:
        # Create database if it doesn't exist
        db = couch.create('rosout')
        print("Created database...")

    # Save messages to database
    def handle_message(msg):
        doc = yaml.load(str(msg))
        db.save(doc)
    
    rospy.init_node('couchdb_logger', anonymous=True)
    sub = rospy.Subscriber('/rosout_agg', Log, handle_message, queue_size=100)
    rospy.spin()