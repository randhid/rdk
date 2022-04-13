// Code generated by protoc-gen-go-grpc. DO NOT EDIT.

package v1

import (
	context "context"
	grpc "google.golang.org/grpc"
	codes "google.golang.org/grpc/codes"
	status "google.golang.org/grpc/status"
)

// This is a compile-time assertion to ensure that this generated file
// is compatible with the grpc package it is being compiled against.
// Requires gRPC-Go v1.32.0 or later.
const _ = grpc.SupportPackageIsVersion7

// RobotServiceClient is the client API for RobotService service.
//
// For semantics around ctx use and closing/ending streaming RPCs, please refer to https://pkg.go.dev/google.golang.org/grpc/?tab=doc#ClientConn.NewStream.
type RobotServiceClient interface {
	// DoAction runs an action on the underlying robot.
	DoAction(ctx context.Context, in *DoActionRequest, opts ...grpc.CallOption) (*DoActionResponse, error)
	// ResourceRunCommand runs an arbitrary command on a resource if it supports it.
	ResourceRunCommand(ctx context.Context, in *ResourceRunCommandRequest, opts ...grpc.CallOption) (*ResourceRunCommandResponse, error)
	GetOperations(ctx context.Context, in *GetOperationsRequest, opts ...grpc.CallOption) (*GetOperationsResponse, error)
	CancelOperation(ctx context.Context, in *CancelOperationRequest, opts ...grpc.CallOption) (*CancelOperationResponse, error)
	BlockForOperation(ctx context.Context, in *BlockForOperationRequest, opts ...grpc.CallOption) (*BlockForOperationResponse, error)
}

type robotServiceClient struct {
	cc grpc.ClientConnInterface
}

func NewRobotServiceClient(cc grpc.ClientConnInterface) RobotServiceClient {
	return &robotServiceClient{cc}
}

func (c *robotServiceClient) DoAction(ctx context.Context, in *DoActionRequest, opts ...grpc.CallOption) (*DoActionResponse, error) {
	out := new(DoActionResponse)
	err := c.cc.Invoke(ctx, "/proto.api.robot.v1.RobotService/DoAction", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *robotServiceClient) ResourceRunCommand(ctx context.Context, in *ResourceRunCommandRequest, opts ...grpc.CallOption) (*ResourceRunCommandResponse, error) {
	out := new(ResourceRunCommandResponse)
	err := c.cc.Invoke(ctx, "/proto.api.robot.v1.RobotService/ResourceRunCommand", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *robotServiceClient) GetOperations(ctx context.Context, in *GetOperationsRequest, opts ...grpc.CallOption) (*GetOperationsResponse, error) {
	out := new(GetOperationsResponse)
	err := c.cc.Invoke(ctx, "/proto.api.robot.v1.RobotService/GetOperations", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *robotServiceClient) CancelOperation(ctx context.Context, in *CancelOperationRequest, opts ...grpc.CallOption) (*CancelOperationResponse, error) {
	out := new(CancelOperationResponse)
	err := c.cc.Invoke(ctx, "/proto.api.robot.v1.RobotService/CancelOperation", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *robotServiceClient) BlockForOperation(ctx context.Context, in *BlockForOperationRequest, opts ...grpc.CallOption) (*BlockForOperationResponse, error) {
	out := new(BlockForOperationResponse)
	err := c.cc.Invoke(ctx, "/proto.api.robot.v1.RobotService/BlockForOperation", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

// RobotServiceServer is the server API for RobotService service.
// All implementations must embed UnimplementedRobotServiceServer
// for forward compatibility
type RobotServiceServer interface {
	// DoAction runs an action on the underlying robot.
	DoAction(context.Context, *DoActionRequest) (*DoActionResponse, error)
	// ResourceRunCommand runs an arbitrary command on a resource if it supports it.
	ResourceRunCommand(context.Context, *ResourceRunCommandRequest) (*ResourceRunCommandResponse, error)
	GetOperations(context.Context, *GetOperationsRequest) (*GetOperationsResponse, error)
	CancelOperation(context.Context, *CancelOperationRequest) (*CancelOperationResponse, error)
	BlockForOperation(context.Context, *BlockForOperationRequest) (*BlockForOperationResponse, error)
	mustEmbedUnimplementedRobotServiceServer()
}

// UnimplementedRobotServiceServer must be embedded to have forward compatible implementations.
type UnimplementedRobotServiceServer struct {
}

func (UnimplementedRobotServiceServer) DoAction(context.Context, *DoActionRequest) (*DoActionResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method DoAction not implemented")
}
func (UnimplementedRobotServiceServer) ResourceRunCommand(context.Context, *ResourceRunCommandRequest) (*ResourceRunCommandResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method ResourceRunCommand not implemented")
}
func (UnimplementedRobotServiceServer) GetOperations(context.Context, *GetOperationsRequest) (*GetOperationsResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetOperations not implemented")
}
func (UnimplementedRobotServiceServer) CancelOperation(context.Context, *CancelOperationRequest) (*CancelOperationResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method CancelOperation not implemented")
}
func (UnimplementedRobotServiceServer) BlockForOperation(context.Context, *BlockForOperationRequest) (*BlockForOperationResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method BlockForOperation not implemented")
}
func (UnimplementedRobotServiceServer) mustEmbedUnimplementedRobotServiceServer() {}

// UnsafeRobotServiceServer may be embedded to opt out of forward compatibility for this service.
// Use of this interface is not recommended, as added methods to RobotServiceServer will
// result in compilation errors.
type UnsafeRobotServiceServer interface {
	mustEmbedUnimplementedRobotServiceServer()
}

func RegisterRobotServiceServer(s grpc.ServiceRegistrar, srv RobotServiceServer) {
	s.RegisterService(&RobotService_ServiceDesc, srv)
}

func _RobotService_DoAction_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(DoActionRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(RobotServiceServer).DoAction(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.robot.v1.RobotService/DoAction",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(RobotServiceServer).DoAction(ctx, req.(*DoActionRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _RobotService_ResourceRunCommand_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(ResourceRunCommandRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(RobotServiceServer).ResourceRunCommand(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.robot.v1.RobotService/ResourceRunCommand",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(RobotServiceServer).ResourceRunCommand(ctx, req.(*ResourceRunCommandRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _RobotService_GetOperations_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(GetOperationsRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(RobotServiceServer).GetOperations(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.robot.v1.RobotService/GetOperations",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(RobotServiceServer).GetOperations(ctx, req.(*GetOperationsRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _RobotService_CancelOperation_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(CancelOperationRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(RobotServiceServer).CancelOperation(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.robot.v1.RobotService/CancelOperation",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(RobotServiceServer).CancelOperation(ctx, req.(*CancelOperationRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _RobotService_BlockForOperation_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(BlockForOperationRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(RobotServiceServer).BlockForOperation(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.robot.v1.RobotService/BlockForOperation",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(RobotServiceServer).BlockForOperation(ctx, req.(*BlockForOperationRequest))
	}
	return interceptor(ctx, in, info, handler)
}

// RobotService_ServiceDesc is the grpc.ServiceDesc for RobotService service.
// It's only intended for direct use with grpc.RegisterService,
// and not to be introspected or modified (even as a copy)
var RobotService_ServiceDesc = grpc.ServiceDesc{
	ServiceName: "proto.api.robot.v1.RobotService",
	HandlerType: (*RobotServiceServer)(nil),
	Methods: []grpc.MethodDesc{
		{
			MethodName: "DoAction",
			Handler:    _RobotService_DoAction_Handler,
		},
		{
			MethodName: "ResourceRunCommand",
			Handler:    _RobotService_ResourceRunCommand_Handler,
		},
		{
			MethodName: "GetOperations",
			Handler:    _RobotService_GetOperations_Handler,
		},
		{
			MethodName: "CancelOperation",
			Handler:    _RobotService_CancelOperation_Handler,
		},
		{
			MethodName: "BlockForOperation",
			Handler:    _RobotService_BlockForOperation_Handler,
		},
	},
	Streams:  []grpc.StreamDesc{},
	Metadata: "proto/api/robot/v1/robot.proto",
}
