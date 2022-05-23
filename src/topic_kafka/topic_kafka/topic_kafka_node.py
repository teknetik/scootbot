from confluent_kafka import Producer, KafkaError, KafkaException
import json
import rclpy
import time
from sensor_msgs.msg import LaserScan

def error_cb(err):
    """ The error callback is used for generic client errors. These
        errors are generally to be considered informational as the client will
        automatically try to recover from all errors, and no extra action
        is typically required by the application.
        For this example however, we terminate the application if the client
        is unable to connect to any broker (_ALL_BROKERS_DOWN) and on
        authentication errors (_AUTHENTICATION). """

    print("Client error: {}".format(err))
    if err.code() == KafkaError._ALL_BROKERS_DOWN or \
       err.code() == KafkaError._AUTHENTICATION:
        # Any exception raised from this callback will be re-raised from the
        # triggering flush() or poll() call.
        raise KafkaException(err)

    # Optional per-message on_delivery handler (triggered by poll() or flush())
    # when a message has been successfully delivered or
    # permanently failed delivery (after retries).
def acked(err, msg):
    global delivered_records
    """Delivery report handler called on
    successful or failed delivery of message
    """
    if err is not None:
        print("Failed to deliver message: {}".format(err))
    else:
        delivered_records += 1
        print("Produced record to topic {} partition [{}] @ offset {}"
                .format(msg.topic(), msg.partition(), msg.offset()))

def kafka_callback(producer, topic, msg):
    print(msg)
    for i in msg:
        print(msg[i])
    #print("Producing record: {}\t{}".format(record_key, record_value))
    #producer.produce(topic, key=record_key, value=record_value, on_delivery=acked)
    #producer.poll(0)


def main():
    print('Hi from topic_kafka.')

    config_file = '/home/teknetik/.confluent/python.config'
    topic = 'rostopic'
    p = Producer({
    'bootstrap.servers': 'pkc-l6wr6.europe-west2.gcp.confluent.cloud:9092',
    'sasl.mechanism': 'PLAIN',
    'security.protocol': 'SASL_SSL',
    'sasl.username': 'WJUBBVOJKCXXGANT',
    'sasl.password': 'SiF2A6Y/NY+zgk+L050hSEseY/8XWquLQ4wIkyLDLm93k2zRK9eOp7bXYgz14Zo1',
    'error_cb': error_cb,
})
    # Create topic if needed
    #ccloud_lib.create_topic(conf, topic)

    delivered_records = 0



    

    global g_node
    rclpy.init(args=None)

    g_node = rclpy.create_node('topic_kafka_node')

    subscription = g_node.create_subscription(LaserScan, 'scan', kafka_callback, 10)
    subscription  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(g_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    producer.flush()
    g_node.destroy_node()
    rclpy.shutdown()

    print("{} messages were produced to topic {}!".format(delivered_records, topic))





if __name__ == '__main__':
    main()
