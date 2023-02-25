class NTGetRaw:
    def __init__(self, ntinst, init, default, failsafe):
        self.init = init
        self.default = default
        self.failsafe = failsafe
        self.table = ntinst.getTable("/Vision")

        self.pub = self.table.getRawTopic(POSE_DATA_RAW_TOPIC_NAME).publish("raw")

    def set(self, raw):
        self.pub.set(raw)

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self.pub.unpublish()

    def close(self):
        # stop subscribing/publishing
        self.pub.close()
