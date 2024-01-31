
# Python program to read
# json file
 
 
import json
 
# Opening JSON file
f = open('sample.json')
 
# returns JSON object as
# a dictionary
data = json.load(f)
 
# Iterating through the json
# list
for i in data['header']:
    print(i)

print((data['poses'][0]))
# Closing file
f.close()