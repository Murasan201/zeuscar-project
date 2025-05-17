import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 0～10の値をキーに、Arduino側が定義しているコマンド文字列をマッピング
COMMANDS_MAP = {
    0:  "FORWARD",
    1:  "BACKWARD",
    2:  "LEFT",
    3:  "RIGHT",
    4:  "LEFTFORWARD",
    5:  "RIGHTFORWARD",
    6:  "LEFTBACKWARD",
    7:  "RIGHTBACKWARD",
    8:  "TURNLEFT",
    9:  "TURNRIGHT",
    10: "STOP"
}

class MinimalPublisher(Node):
    """
    文字列メッセージを送信するROS 2パブリッシャノード
    """
    def __init__(self):
        # ノード名 'minimal_publisher' を設定して初期化
        super().__init__('minimal_publisher')

        # String型メッセージを扱うパブリッシャを作成
        # 第1引数: メッセージ型
        # 第2引数: トピック名 ('topic')
        # 第3引数: キューサイズ
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # ノードが起動したことをログ出力
        self.get_logger().info('MinimalPublisher node has been started.')

    def publish_command(self, command_str: str):
        """
        Arduino側で定義された文字列コマンドをROS 2トピックへPublish
        """
        # 送信用のStringメッセージ生成
        msg = String()
        msg.data = command_str

        # パブリッシャを使って送信
        self.publisher_.publish(msg)

        # 送信した内容をログ出力
        self.get_logger().info(f'Publishing command: "{command_str}"')

def main(args=None):
    # ROS 2通信を初期化
    rclpy.init(args=args)

    # MinimalPublisherノードをインスタンス化
    minimal_publisher = MinimalPublisher()

    try:
        # rclpy.ok() が Trueの間、処理を続行
        while rclpy.ok():
            # spin_onceでROS内部処理をポーリングしつつ、すぐに返す
            rclpy.spin_once(minimal_publisher, timeout_sec=0.1)

            # ユーザーに数値（0～10）を入力してもらう
            user_input = input(
                "Enter command number (0=FORWARD, 1=BACKWARD, etc.) or 'exit': "
            ).strip()

            # 'exit' で入力ループを終了
            if user_input.lower() == 'exit':
                print("Exiting command input loop.")
                break

            # 入力された値が数値かどうか判定
            if user_input.isdigit():
                cmd_num = int(user_input)
                # 0～10の範囲なら対応する文字列を取得してPublish
                if cmd_num in COMMANDS_MAP:
                    command_str = COMMANDS_MAP[cmd_num]
                    minimal_publisher.publish_command(command_str)
                else:
                    print("Invalid number. Please enter a value between 0 and 10.")
            else:
                # 数字以外（exit以外）の入力はエラー扱いとする
                print("Invalid input. Please enter a number (0?10) or 'exit'.")

    except KeyboardInterrupt:
        pass
    finally:
        # ノードを明示的に破棄し、rclpyをシャットダウン
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
