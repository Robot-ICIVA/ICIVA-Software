import requests

# Define the IP address and Port of the Server.
ip_address = "127.0.0.1"
port = "8000"
# Make the request
r = requests.get("http://" + ip_address + ":" + port)


# Move the response to another variable
response = r.json()
# Print the response
print(response)
