from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import time
import cv2


"""Define carry load and takeoff"""
def carry_and_takeoff(altitude):

    while not vehicle.is_armable:
        print("waiting to be loaded")
        time.sleep(2)

    print("Putting load")
    airvan.mode = VehicleMode("GUIDED")
    airvan.armed = True

    while not airvan.armed: time.sleep(1)

    print("Lifting up!")
    airvan.simple_takeoff(altitude)

    while True:
        v_alt = airvan.location.global_relative_frame.alt
        print(">> Altitude = %.1f m"%v_alt)
        if v_alt >= altitude - 2.0:
            print("Target altitude reached")
            break
        time.sleep(1)

def clear_mission(airvan):
    """
    Clear the current mission.
    """
    cmds = airvan.commands
    airvan.commands.clear()
    airvan.flush()
    cmds = airvan.commands
    cmds.download()
    cmds.wait_ready()

def download_job(airvan):
    """Download the current job."""
    cmds = airvan.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def get_current_job(airvan):
    """ Downloads the mission and returns the wp list and number of Waypoints"""
    print("Downloading mission")
    download_job(airvan)
    newjobwork = []
    n_WP= 0
    for wp in airvan.commands:
        newjobwork.append(wp)
        n_WP += 1

    return n_WP, newjobwork


def add_waypoint_to_work(airvan,wp_Last_Latitude,wp_Last_Longitude,wp_Last_Altitude):
    """ Upload the Job along with the waypoint """
    cmds = airvan.commands
    cmds.download()
    cmds.wait_ready()
    joblist=[]
    for cmd in cmds:
        joblist.append(cmd)


    wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                            wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    joblist.append(wpLastObject)
    cmds.clear()


    for cmd in joblist:
        cmds.add(cmd)
    cmds.upload()

    return (cmds.count)

def ChangeMode(airvan, mode):
    while airvan.mode != VehicleMode(mode):
        airvan.mode = VehicleMode(mode)
        time.sleep(0.5)
    return True

gnd_speed = 10 # [m/s]
mode      = 'GROUND'


print('Establishing Connection...')
airvan = connect('udp:127.0.0.1:14551')

while True:
    airvan.stream()
    vals=get_current_job()
    airvan.send_rc_control(vals[0],vals[1],vals[2],vals[3])
    img=airvan.get_frame_read().frame
    img=cv2.resize(img,(360,240))
    cv2.imshow("image",img)
    cv2.Waitkey(1)

    if mode == 'GROUND':

        n_WP, newjobwork = get_current_job(airvan)
        time.sleep(2)
        if n_WP > 0:
            print ("A valid mission has been uploaded: takeoff!")
            mode = 'TAKEOFF'

    elif mode == 'TAKEOFF':


        add_waypoint_to_work(airvan, airvan.location.global_relative_frame.lat,
                                     airvan.location.global_relative_frame.lon,
                                     airvan.location.global_relative_frame.alt)
        print("Home waypoint added to the work")
        time.sleep(2)
        carry_and_takeoff(15)
        print("Changing the mode to AUTO")
        ChangeMode(airvan,"AUTO")
        airvan.groundspeed = gnd_speed
        mode = 'MISSION'
        print ("Switch mode to work")

    elif mode == 'MISSION':
        print ("Current WP: %d of %d "%(airvan.commands.next, airvan.commands.count))
        if airvan.commands.next == airvan.commands.count:
            print ("Final waypoint reached: go back home")
            clear_mission(airvan)
            print ("Mission deleted")
            ChangeMode(airvan,"RTL")
            mode = "BACK"

    elif mode == "BACK":
        if airvan.location.global_relative_frame.alt < 1:
            print ("Switch to GROUND mode, waiting for new job")
            mode = 'GROUND'




    time.sleep(0.5)
