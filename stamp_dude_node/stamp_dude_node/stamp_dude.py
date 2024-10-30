import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import subprocess
import importlib
import sys

class StampDudeNode(Node):
    def __init__(self):
        super().__init__('dynamic_subscriber')

        # パラメータからトピック名を取得
        self.declare_parameter('topic_name', '/default_topic')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        # トピックの型を取得
        topic_type = None
        while topic_type == None and rclpy.ok():
            topic_type = self.get_topic_type(topic_name)

        # スラッシュ区切りからピリオド区切りに変換
        topic_type = topic_type.replace('/', '.')

        # 指定された型のメッセージタイプを使ってサブスクライバを作成
        msg_module = importlib.import_module(topic_type.rsplit('.', 1)[0])
        msg_class = getattr(msg_module, topic_type.rsplit('.', 1)[1])

        # メッセージのフィールド名を取得
        self.field_names = msg_class.get_fields_and_field_types().keys() if hasattr(msg_class, 'get_fields_and_field_types') else msg_class.__slots__

        try:
            # メッセージタイプを動的にインポートし、サブスクライバを作成
            stamped_msg_class = getattr(msg_module, topic_type.rsplit('.', 1)[1] + 'Stamped')
            # メッセージのフィールド名を取得
            stamped_field_names = stamped_msg_class.get_fields_and_field_types().keys() if hasattr(stamped_msg_class, 'get_fields_and_field_types') else stamped_msg_class.__slots__
            for field in stamped_field_names:
                if not field == 'header':
                    self.target_field_name = field
                    break
        except AttributeError:
            self.get_logger().error(f"Message class '{msg_class_name}' not found in module '{msg_module.__name__}'")
            sys.exit(1)  # エラー終了
        
        # サブスクライバとパブリッシャの設定
        self.subscription = self.create_subscription(
            msg_class,
            topic_name,
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            stamped_msg_class,
            topic_name + '_stamped',
            10
        )
        self.get_logger().info(f'Subscribed to topic: {topic_name} and publishing to: {topic_name}_stamped')

    def get_topic_type(self, topic_name):
        # `ros2 topic info` コマンドでトピックの型を取得
        result = subprocess.run(['ros2', 'topic', 'info', topic_name],
                                stdout=subprocess.PIPE,
                                text=True)
        for line in result.stdout.splitlines():
            if 'Type:' in line:
                return line.split('Type: ')[1]
        return None

    def listener_callback(self, msg):
        # 新しい Stamped メッセージを作成
        stamped_msg = self.publisher.msg_type()
        stamped_msg.header = Header()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 元のメッセージ内容をコピー
        # 各フィールドをコピー
        for field in self.field_names:
            value = getattr(msg, field)  # 元のメッセージのフィールド値を取得
            target_attr = getattr(stamped_msg, self.target_field_name)  # 元のメッセージのフィールド値を取得
            setattr(target_attr, field, value)  # Stampedメッセージの同名フィールドに設定

        self.publisher.publish(stamped_msg)
        self.get_logger().info('Published Stamped message')

def main(args=None):
    rclpy.init(args=args)
    node = StampDudeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
