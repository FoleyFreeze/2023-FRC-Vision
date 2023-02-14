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

def main():
    
    # start NetworkTables
    ntconnect = NTConnectType(NTConnectType.SERVER)
    ntinst = NetworkTableInstance.getDefault()
    if ntconnect == NTConnectType.SERVER:
        ntinst.startServer()
    else:
        ntinst.startClient4("raspberrypi910")
 
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

    # Wait for NetworkTables to start
    time.sleep(0.5)
    
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
    calib_data = np.load(f"{calib_data_path}/MultiMatrix.npz")
    camMatrix = calib_data["camMatrix"]
    distCoeffs = calib_data["distCoef"]

    apriltag_est_config = robotpy_apriltag.AprilTagPoseEstimator.Config(0.153, camMatrix[0][0], camMatrix[1][1], camMatrix[0][2], camMatrix[1][2])
    apriltag_est = robotpy_apriltag.AprilTagPoseEstimator(apriltag_est_config)
    
    # Capture from the first USB Camera on the system
    camera = CameraServer.startAutomaticCapture()
    camera.setResolution(X_RES, Y_RES)

    # Get a CvSink. This will capture images from the camera
    cvSink = CameraServer.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = CameraServer.putVideo("final image", X_RES, Y_RES)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(X_RES, Y_RES, 3), dtype=np.uint8)

    print("Hello")

    seconds = 0
    current_seconds = 0
    prev_seconds = 0
    while True:
        
        start_time = time.time()
        current_seconds = start_time
        if int(current_seconds - prev_seconds) >= SECOND_COUNTER:
            prev_seconds = current_seconds
            seconds = seconds + 1
            uptime_ntt.set(seconds)
            print(seconds)

        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        frame_time, img = cvSink.grabFrame(img)
        if frame_time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue
        #
        # Insert your image processing logic here!
        #
        gimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if debug_ntt.get() == True:
            detectorConfig.numThreads = int(threads_ntt.get())
            detectorConfig.quadDecimate = float(quadDecimate_ntt.get())
            detectorConfig.quadSigma = float(blur_ntt.get())
            detectorConfig.refineEdges = refineEdges_ntt.get()
            detectorConfig.decodeSharpening = float(decodeSharpening_ntt.get())
            detectorConfig.debug = ATDebug_ntt.get()
            detector.setConfig(detectorConfig)

        detected = detector.detect(gimg)
        for tag in detected:
            if tag.getDecisionMargin() > float(config.get('VISION', DECISION_MARGIN_TOPIC_NAME)) and tag.getId() >= 1 and tag.getId() <= 8:
                tag_pose = apriltag_est.estimate(tag)
                if debug_ntt.get() == True:
                
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
                    #cv2.drawFrameAxes(img, camMatrix, distCoeffs, tag_pose.rotation(), tag_pose.translation(), 76, 3)

        if debug_ntt.get() == True:
            outputStream.putFrame(img) # send to dashboard
            if savefile_ntt.get() == True:
                file_write(configfile_ntt.get(), threads_ntt.get(), \
                    quadDecimate_ntt.get(), blur_ntt.get(), refineEdges_ntt.get(), \
                    decodeSharpening_ntt.get(), ATDebug_ntt.get(), \
                    decision_margin_ntt.get())
                savefile_ntt.set(False)

main()