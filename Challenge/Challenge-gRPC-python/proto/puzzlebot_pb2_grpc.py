# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2
import puzzlebot_pb2 as puzzlebot__pb2


class PuzzlebotOdometryStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.GetOdometry = channel.unary_unary(
                '/RPCDemoPkg.PuzzlebotOdometry/GetOdometry',
                request_serializer=google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
                response_deserializer=puzzlebot__pb2.Odometry.FromString,
                )


class PuzzlebotOdometryServicer(object):
    """Missing associated documentation comment in .proto file."""

    def GetOdometry(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_PuzzlebotOdometryServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'GetOdometry': grpc.unary_unary_rpc_method_handler(
                    servicer.GetOdometry,
                    request_deserializer=google_dot_protobuf_dot_empty__pb2.Empty.FromString,
                    response_serializer=puzzlebot__pb2.Odometry.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'RPCDemoPkg.PuzzlebotOdometry', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class PuzzlebotOdometry(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def GetOdometry(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/RPCDemoPkg.PuzzlebotOdometry/GetOdometry',
            google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
            puzzlebot__pb2.Odometry.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
