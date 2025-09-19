Drone Simulator
Simulates 20 drones sending MAVLink telemetry to a Java backend on a remote PC and displays positions on a web UI using WebSockets.
Setup

Install Python 3.8+.
Create and activate a virtual environment:python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate


Install dependencies:pip install -r requirements.txt


Update HOST in app.py to the IP address of the Java backend PC (e.g., 192.168.1.123).
Ensure the Java backend is running on the remote PC, listening on UDP ports 15001–15020 (optional for UI).
Open firewall ports 15001–15020/UDP on the backend PC:
Linux: sudo ufw allow 15001:15020/udp
Windows: Add inbound rule for UDP 15001–15020.


Run the Flask app:python app.py


Open http://localhost:5000 in a browser to view the UI.

Dependencies

Flask==2.2.5
pymavlink==2.4.40
geopy==2.4.1
Flask-SocketIO==5.3.6

Notes

Sends MAVLink messages to <HOST>:15001–15020 via UDP.
UI uses WebSockets for real-time telemetry updates.
Ensure both PCs are on the same network and UDP ports are open (if backend is used).
The UI uses Leaflet via CDN for map rendering.
If markers don’t appear, check browser console (F12) for errors and verify internet for Leaflet CDN.
Debug connection issues with ping <HOST> or Wireshark.
# simulation-test-program
