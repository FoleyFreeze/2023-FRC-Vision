def pose_data_string(sequence_num, rio_time, time, tags, tag_poses):
    string_header = ""
    string_header = f'num={sequence_num} t_rio={rio_time:1.3f} t_img={time:1.3f} len={len(tags)}'

    string_data_rot = f'tags={len(tags)} '
    string_data_t = f'tags={len(tags)} '
    tag_pose = 0
    
    for tag in tags:
        
        z_in = (tag_poses[tag_pose].translation().Z() * 39.3701)

        string_data_rot += f'id={tag.getId()} \
        x_deg={math.degrees(tag_poses[tag_pose].rotation().X()):3.1f} \
        y_deg={math.degrees(tag_poses[tag_pose].rotation().Y()):3.1f} \
        z_deg={math.degrees(tag_poses[tag_pose].rotation().Z()):3.1f} '
        string_data_t += f'id={tag.getId()} \
        x_in={(tag_poses[tag_pose].translation().X() * 39.37):3.2f} \
        y_in={(tag_poses[tag_pose].translation().Y() * 39.37):3.2f} \
        z_in={(tag_poses[tag_pose].translation().Z() * 39.37):3.2f} '
        tag_pose +=1
    
    return string_header, string_data_rot, string_data_t, z_in

def draw_tags(img, tags, tag_poses, rVector, tVector, camMatrix, distCoeffs):
    tag_pose = 0
    
    for tag in tags:
        x0 = int(tag.getCorner(0).x)
        y0 = int(tag.getCorner(0).y)
        x1 = int(tag.getCorner(1).x)
        y1 = int(tag.getCorner(1).y)
        x2 = int(tag.getCorner(2).x)
        y2 = int(tag.getCorner(2).y)
        x3 = int(tag.getCorner(3).x)
        y3 = int(tag.getCorner(3).y)
        cv2.line(img, (x0, y0), (x1, y1), (0,255,0), 5) #starts at top left corner of apriltag
        cv2.line(img, (x1, y1), (x2, y2), (0,255,0), 5) #top left to bottom left
        cv2.line(img, (x2, y2), (x3, y3), (0,255,0), 5) #bottom left to bottom right
        cv2.line(img, (x3, y3), (x0, y0), (0,255,0), 5) #bottom right to top right
        cv2.putText(img, str(tag.getId()), (int(tag.getCenter().x), int(tag.getCenter().y)), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255)) # ID in center
        rVector[0][0] = tag_poses[tag_pose].rotation().X()
        rVector[1][0] = tag_poses[tag_pose].rotation().Y()
        rVector[2][0] = tag_poses[tag_pose].rotation().Z()
        tVector[0][0] = tag_poses[tag_pose].translation().X()
        tVector[1][0] = tag_poses[tag_pose].translation().Y()
        tVector[2][0] = tag_poses[tag_pose].translation().Z()
        tag_pose += 1
        #for rotation, ask if its shrinking on each axis 
        cv2.drawFrameAxes(img, camMatrix, distCoeffs, rVector, tVector, .076, 3)
    return img

def file_write(file, threads, decimate, blur, refine, sharpen, atdebug, decisionmargin):

    parser = configparser.ConfigParser()

    parser.add_section('VISION')
    parser.set('VISION', THREADS_TOPIC_NAME, str(threads))
    parser.set('VISION', BLUR_TOPIC_NAME, str(blur))
    parser.set('VISION', REFINE_EDGES_TOPIC_NAME, str(refine))
    parser.set('VISION', SHARPENING_TOPIC_NAME, str(sharpen))
    parser.set('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME, str(atdebug))
    parser.set('VISION', DECISION_MARGIN_TOPIC_NAME, str(decisionmargin))
    parser.set('VISION', DECIMATE_TOPIC_NAME, str(decimate))
    parser.set('VISION', CONFIG_FILE_TOPIC_NAME, str(file))
    #HEY HEY HEY!!! LOOK AT MEEEEE!!!! >>>pscp.exe pi@10.2.33.177:/home/pi/config.ini C:\Users\23JMurphy\Downloads will copy any file from pi to windows<<<
    with open(file, 'w') as config:
        parser.write(config)

    with open('Config_file_name_holder_file', 'w') as container:
        container.write(file)

def file_read(parser, configfile_failure_ntt):
    container_exists = os.path.isfile('Config_file_name_holder_file') #normal file read case
    if container_exists == True:
        container = open('Config_file_name_holder_file', 'r')
        config_file = container.readline()
        container.close()
        config_exists = os.path.isfile(config_file)
        if config_exists == True:
            parser.read(config_file)
            configfile_failure_ntt.set(False) #if it works mark no error
    else: # re-create config and container file to default
        configfile_failure_ntt.set(True) # set error for config file

        parser.add_section('VISION')
        parser.set('VISION', THREADS_TOPIC_NAME, str(THREADS_DEFAULT))
        parser.set('VISION', BLUR_TOPIC_NAME, str(BLUR_DEFAULT))
        parser.set('VISION', REFINE_EDGES_TOPIC_NAME, str(REFINE_EDGES_DEFAULT))
        parser.set('VISION', SHARPENING_TOPIC_NAME, str(SHARPENING_DEFAULT))
        parser.set('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME, str(APRILTAG_DEBUG_MODE_DEFAULT))
        parser.set('VISION', DECISION_MARGIN_TOPIC_NAME, str(DECISION_MARGIN_DEFAULT))
        parser.set('VISION', DECIMATE_TOPIC_NAME, str(DECIMATE_DEFAULT))
        parser.set('VISION', CONFIG_FILE_TOPIC_NAME, str(CONFIG_FILE_DEFAULT))

        with open("/home/pi/" + CONFIG_FILE_DEFAULT, 'w') as config:
            parser.write(config)

        with open("/home/pi/" + 'Config_file_name_holder_file', 'w') as container:
            container.write(str(CONFIG_FILE_DEFAULT))
    
def nt_update(config, threads,quadDecimate, blur, refineEdges, decodeSharpening, \
     ATDebug, decision, configfile):
    # sync the stuff in the file with matching values in the file

    threads.set(float(config.get('VISION', THREADS_TOPIC_NAME)))
    quadDecimate.set(float(config.get('VISION', DECIMATE_TOPIC_NAME)))
    blur.set(float(config.get('VISION', BLUR_TOPIC_NAME)))
    refineEdges.set(ast.literal_eval(config.get('VISION', REFINE_EDGES_TOPIC_NAME)))
    decodeSharpening.set(float(config.get('VISION', SHARPENING_TOPIC_NAME)))
    ATDebug.set(ast.literal_eval(config.get('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME)))
    decision.set(float(config.get('VISION', DECISION_MARGIN_TOPIC_NAME)))
    configfile.set(str(config.get('VISION', CONFIG_FILE_TOPIC_NAME)))

'''
all data to send is packaged as an array of bytes, using a Python bytearray, in big-endian format:
sequence number: unsigned long (4 bytes)
rio time: float (4 bytes)
image time:float (4 bytes)
type (tag = 1, cone = 2, cube = 3): unsigned char (1 byte)
length: how many tags/cones/cubes follow
what follows these first 3 items depends on the type:
tag:
number of tags detected: unsigned char (1 byte)
for each tag: tag id unsigned char (1 byte), pose x: float (4 bytes), pose y: float (4 bytes), pose z: float (4 bytes), pose x angle: float (4 bytes), pose y angle: float (4 bytes), pose z angle: float (4 bytes)
cone:
number of cones detected: unsigned char (1 byte)
for each cone: pose x: float (4 bytes), pose y: float (4 bytes), pose z: float (4 bytes), pose x angle: float (4 bytes), pose y angle: float (4 bytes), pose z angle: float (4 bytes)
cube:
number of cubes detected: unsigned char (1 byte)
for each cube: pose x: float (4 bytes), pose y: float (4 bytes), pose z: float (4 bytes), pose x angle: float (4 bytes), pose y angle: float (4 bytes), pose z angle: float (4 bytes)
'''
def pose_data_bytes(sequence_num, rio_time, image_time, tags, tag_poses):
    byte_array = bytearray()
    # get a list of tags that were detected
    # start the array with sequence number, the RIO's time, image time, and tag type
    tag_pose = 0
    byte_array += struct.pack(">LffBB", sequence_num, rio_time, image_time, 1, len(tags))
    for tag in tags:
        byte_array += struct.pack(">Bffffff", tag.getId(), \
            tag_poses[tag_pose].rotation().X(), tag_poses[tag_pose].rotation().Y(), tag_poses[tag_pose].rotation().Z(), \
            tag_poses[tag_pose].translation().X(), tag_poses[tag_pose].translation().Y(), tag_poses[tag_pose].translation().Z())
        tag_pose += 1
    return byte_array

def main():
    
    # start NetworkTables
    ntconnect = NTConnectType(NTConnectType.SERVER)
    ntinst = NetworkTableInstance.getDefault()
    if ntconnect == NTConnectType.SERVER:
        ntinst.startServer()
    else:
        print("connect as client")
        ntinst.startClient4("raspberrypi910")
        ntinst.setServerTeam(910)
 
    # Wait for NetworkTables to start
    time.sleep(1)
    
    rio_time_ntt = NTGetDouble(ntinst.getDoubleTopic(RIO_TIME_TOPIC_NAME), 0, 0, 0)
    
    if ntconnect == NTConnectType.CLIENT:
        while rio_time_ntt.get() == 0:
            time.sleep(1)
            print("waiting for RIO connection")

    # Table for vision output information
    uptime_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Uptime"), 0, 0, -1)
    #debug_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Debug Mode"), False, DEBUG_MODE_DEFAULT, DEBUG_MODE_DEFAULT)
    debug_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Debug Mode"), True, True, True)
    threads_ntt = NTGetDouble(ntinst.getDoubleTopic(THREADS_TOPIC_NAME),THREADS_DEFAULT, THREADS_DEFAULT, THREADS_DEFAULT)
    quadDecimate_ntt = NTGetDouble(ntinst.getDoubleTopic(DECIMATE_TOPIC_NAME),DECIMATE_DEFAULT, DECIMATE_DEFAULT, DECIMATE_DEFAULT)
    blur_ntt = NTGetDouble(ntinst.getDoubleTopic(BLUR_TOPIC_NAME),BLUR_DEFAULT, BLUR_DEFAULT, BLUR_DEFAULT) 
    refineEdges_ntt = NTGetBoolean(ntinst.getBooleanTopic(REFINE_EDGES_TOPIC_NAME),REFINE_EDGES_DEFAULT, REFINE_EDGES_DEFAULT, REFINE_EDGES_DEFAULT) 
    decodeSharpening_ntt = NTGetDouble(ntinst.getDoubleTopic(SHARPENING_TOPIC_NAME), SHARPENING_DEFAULT, SHARPENING_DEFAULT, SHARPENING_DEFAULT)
    ATDebug_ntt = NTGetBoolean(ntinst.getBooleanTopic(APRILTAG_DEBUG_MODE_TOPIC_NAME), APRILTAG_DEBUG_MODE_DEFAULT, APRILTAG_DEBUG_MODE_DEFAULT, APRILTAG_DEBUG_MODE_DEFAULT)
    decision_margin_ntt = NTGetDouble(ntinst.getDoubleTopic(DECISION_MARGIN_TOPIC_NAME), DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT)
    configfile_ntt = NTGetString(ntinst.getStringTopic(CONFIG_FILE_TOPIC_NAME), CONFIG_FILE_DEFAULT, CONFIG_FILE_DEFAULT, CONFIG_FILE_DEFAULT)
    savefile_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Save File"), False, False, False)
    configfilefail_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Config File Fail"), False, False, False)
    active_ntt = NTGetBoolean(ntinst.getBooleanTopic(ACTIVE_TOPIC_NAME), True, True, True)
    pose_data_bytes_ntt = NTGetRaw(ntinst, None, None, None)
    pose_data_string_header_ntt = NTGetString(ntinst.getStringTopic(POSE_DATA_STRING_TOPIC_NAME_HEADER),"", "", "")
    pose_data_string_data_translation_ntt = NTGetString(ntinst.getStringTopic(POSE_DATA_STRING_TOPIC_NAME_DATA_TRANSLATION),"", "", "")
    pose_data_string_data_rotation_ntt = NTGetString(ntinst.getStringTopic(POSE_DATA_STRING_TOPIC_NAME_DATA_ROTATION),"", "", "")
    temp_ntt = NTGetDouble(ntinst.getDoubleTopic(TEMP_TOPIC_NAME), 0, 0, 0)
    z_in_ntt = NTGetDouble(ntinst.getDoubleTopic(Z_IN_TOPIC_NAME), 0.0, 0.0, 0.0)

    detector = robotpy_apriltag.AprilTagDetector()
    detector.addFamily("tag16h5")

    # use for file
    config = configparser.ConfigParser()
    file_read(config, configfilefail_ntt)
    nt_update(config,threads_ntt, quadDecimate_ntt, blur_ntt, refineEdges_ntt, \
        decodeSharpening_ntt, ATDebug_ntt, decision_margin_ntt, configfile_ntt)
    detectorConfig = robotpy_apriltag.AprilTagDetector.Config()

    detectorConfig.numThreads = int(float(config.get('VISION', THREADS_TOPIC_NAME)))
    detectorConfig.quadDecimate = float(config.get('VISION', DECIMATE_TOPIC_NAME))
    detectorConfig.quadSigma = float (config.get('VISION', BLUR_TOPIC_NAME))
    detectorConfig.refineEdges = ast.literal_eval(config.get('VISION', REFINE_EDGES_TOPIC_NAME))
    detectorConfig.decodeSharpening = float(config.get('VISION', DECISION_MARGIN_TOPIC_NAME))
    detectorConfig.debug = ast.literal_eval(config.get('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME))
    detector.setConfig(detectorConfig)
    
    #set up pose estimation
    calib_data_path = "calib_data"
    calib_data = np.load(f"{calib_data_path}/{CAMERA_CAL_FILE_NAME}")
    camMatrix = calib_data["camMatrix"]
    distCoeffs = calib_data["distCoef"]

    #camMatrix[0][0] = Focal point distance x (fx) 
    #camMatrix[1][1] = Focal point distance y (fy) 
    #camMatrix[0][2] = camera center  (cx) 
    #camMatrix[1][2] = camera center  (cy) 

    apriltag_est_config = robotpy_apriltag.AprilTagPoseEstimator.Config(0.153, camMatrix[0][0], camMatrix[1][1], camMatrix[0][2], camMatrix[1][2])
    apriltag_est = robotpy_apriltag.AprilTagPoseEstimator(apriltag_est_config)
    rVector = np.zeros((3,1))
    tVector = np.zeros((3,1))
    
    #load camera settings set from web console
    with open('/boot/frc.json') as f:
        web_settings = json.load(f)
    cam_config = web_settings['cameras'][0]

    # Capture from the first USB Camera on the system
    #camera = CameraServer.startAutomaticCapture()

    w = cam_config['width']
    h = cam_config['height']
    #camera.setResolution(w, h)
    fps = cam_config['fps']
    # Get a CvSink. This will capture images from the camera
    #cvSink = CameraServer.getVideo()

    picam2 = Picamera2()
    picam2_config = picam2.create_still_configuration({"size": (w, h)})
    picam2.still_configuration.controls.FrameRate = fps
    print(picam2_config["main"])
    picam2.configure(picam2_config)
    picam2.start()

    
    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = CameraServer.putVideo("final image", cam_config['width'], cam_config['height'])

    # Allocating new images is very expensive, always try to preallocate
    #img = np.zeros(shape=(cam_config['height'], cam_config['width'], 3), dtype=np.uint8)

    print("Hello")
    image_num = 0
    seconds = 0
    current_seconds = 0
    prev_seconds = 0
    temp_sec = 30
    while True:
        rio_time = rio_time_ntt.get()
        start_time = time.time()
        current_seconds = start_time
        if current_seconds - prev_seconds >= UPTIME_UPDATE_INTERVAL:
            prev_seconds = current_seconds
            seconds = seconds + 1
            temp_sec = temp_sec + 1
            uptime_ntt.set(seconds)
            print(f'w={w} h={h} sec={seconds}')
        
        if temp_sec >= TEMP_UPDATE_INTERVAL:
            with open("/sys/class/thermal/thermal_zone0/temp", 'r') as f:
                temp_ntt.set(int(f.readline()) / 1000) #converting milidegrees C to degrees C
                temp_sec = 0
        
        if active_ntt.get() == True:
            # Tell the CvSink to grab a frame from the camera and put it
            # in the source image.  If there is an error notify the output.
            
            t1_time = time.process_time()
            #frame_time, img = cvSink.grabFrame(img)
            img = picam2.capture_array()
            '''
            if frame_time == 0:
                # Send the output the error.
                outputStream.notifyError(cvSink.getError())
                # skip the rest of the current iteration
                continue
            '''
            #
            # Insert your image processing logic here!
            #
            #img = cv2.flip(img, -1)
            gimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            if debug_ntt.get() == True:
                detectorConfig.numThreads = int(threads_ntt.get())
                detectorConfig.quadDecimate = float(quadDecimate_ntt.get())
                detectorConfig.quadSigma = float(blur_ntt.get())
                detectorConfig.refineEdges = refineEdges_ntt.get()
                detectorConfig.decodeSharpening = float(decodeSharpening_ntt.get())
                detectorConfig.debug = ATDebug_ntt.get()
                detector.setConfig(detectorConfig)
                config.set('VISION', DECISION_MARGIN_TOPIC_NAME, str(decision_margin_ntt.get()))

            detected = detector.detect(gimg)
            tag_poses = []
            tags = []
            for tag in detected:
                #print(f'num={len(tags)} DM={tag.getDecisionMargin()}')
                if tag.getDecisionMargin() > float(config.get('VISION', DECISION_MARGIN_TOPIC_NAME)) and tag.getId() >= 1 and tag.getId() <= 8:
                    tag_pose = apriltag_est.estimateHomography(tag)
                    tag_poses.append(tag_pose)
                    tags.append(tag)
                    
            if len(tags) > 0:
                image_num += 1
                image_time = time.process_time() - t1_time
                pose_data = pose_data_bytes(image_num, rio_time, image_time, tags, tag_poses)
                pose_data_bytes_ntt.set(pose_data)
                header, rot_data, trans_data, z_in = pose_data_string(image_num, rio_time, image_time, tags, tag_poses)
                z_in_ntt.set(z_in)
                pose_data_string_header_ntt.set(header)

            if debug_ntt.get() == True:
                if len(tags) > 0:
                    img = draw_tags(img, tags, tag_poses, rVector, tVector, camMatrix, distCoeffs)
                    header, rot_data, trans_data, z_in = pose_data_string(image_num, rio_time, image_time, tags, tag_poses)
                    pose_data_string_header_ntt.set(header)
                    pose_data_string_data_translation_ntt.set(trans_data)
                    pose_data_string_data_rotation_ntt.set(rot_data)
                    z_in_ntt.set(z_in)
                    NetworkTableInstance.getDefault().flush()
                outputStream.putFrame(img) # send to dashboard
                if savefile_ntt.get() == True:
                    file_write(configfile_ntt.get(), threads_ntt.get(), \
                        quadDecimate_ntt.get(), blur_ntt.get(), refineEdges_ntt.get(), \
                        decodeSharpening_ntt.get(), ATDebug_ntt.get(), \
                        decision_margin_ntt.get())
                    savefile_ntt.set(False)

main()