#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hospital_robot_pkg.srv import QueryRAG
import sys

class RAGClientNode(Node):

    def __init__(self):
        super().__init__('rag_client_node')
        # 'query_rag_service' 이름으로 서비스 클라이언트 생성
        self.client = self.create_client(QueryRAG, 'query_rag_service')
        
        # 서비스 서버가 준비될 때까지 1초마다 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중... (데스크탑에서 rag_server가 실행 중인지 확인하세요)')
        
        self.req = QueryRAG.Request()

    def send_request(self, question):
        self.req.question = question
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    # 커맨드 라인에서 질문을 받음
    if len(sys.argv) < 2:
        print("사용법: ros2 run hospital_robot_pkg robot_client '질문 내용'")
        return

    question = " ".join(sys.argv[1:])
    
    rag_client_node = RAGClientNode()
    rag_client_node.get_logger().info(f'"{question}"에 대해 RAG 서비스 요청 중...')
    
    response = rag_client_node.send_request(question)
    
    if response:
        # 응답을 터미널에 예쁘게 출력
        rag_client_node.get_logger().info(f'\n--- RAG 서비스 답변 ---\n{response.answer}')
    else:
        rag_client_node.get_logger().error('서비스 호출 실패')

    rag_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()