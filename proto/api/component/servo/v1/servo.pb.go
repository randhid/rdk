// Code generated by protoc-gen-go. DO NOT EDIT.
// versions:
// 	protoc-gen-go v1.27.1
// 	protoc        (unknown)
// source: proto/api/component/servo/v1/servo.proto

package v1

import (
	_ "google.golang.org/genproto/googleapis/api/annotations"
	protoreflect "google.golang.org/protobuf/reflect/protoreflect"
	protoimpl "google.golang.org/protobuf/runtime/protoimpl"
	reflect "reflect"
	sync "sync"
)

const (
	// Verify that this generated code is sufficiently up-to-date.
	_ = protoimpl.EnforceVersion(20 - protoimpl.MinVersion)
	// Verify that runtime/protoimpl is sufficiently up-to-date.
	_ = protoimpl.EnforceVersion(protoimpl.MaxVersion - 20)
)

type MoveRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// the name of the servo, as registered
	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
	// the degrees by which to rotate the servo. Accepted values are between 0 and 180
	AngleDeg uint32 `protobuf:"varint,2,opt,name=angle_deg,json=angleDeg,proto3" json:"angle_deg,omitempty"`
}

func (x *MoveRequest) Reset() {
	*x = MoveRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_servo_v1_servo_proto_msgTypes[0]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *MoveRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*MoveRequest) ProtoMessage() {}

func (x *MoveRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_servo_v1_servo_proto_msgTypes[0]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use MoveRequest.ProtoReflect.Descriptor instead.
func (*MoveRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_servo_v1_servo_proto_rawDescGZIP(), []int{0}
}

func (x *MoveRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

func (x *MoveRequest) GetAngleDeg() uint32 {
	if x != nil {
		return x.AngleDeg
	}
	return 0
}

type MoveResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields
}

func (x *MoveResponse) Reset() {
	*x = MoveResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_servo_v1_servo_proto_msgTypes[1]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *MoveResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*MoveResponse) ProtoMessage() {}

func (x *MoveResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_servo_v1_servo_proto_msgTypes[1]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use MoveResponse.ProtoReflect.Descriptor instead.
func (*MoveResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_servo_v1_servo_proto_rawDescGZIP(), []int{1}
}

type GetPositionRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// the name of the servo, as registered
	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
}

func (x *GetPositionRequest) Reset() {
	*x = GetPositionRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_servo_v1_servo_proto_msgTypes[2]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *GetPositionRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*GetPositionRequest) ProtoMessage() {}

func (x *GetPositionRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_servo_v1_servo_proto_msgTypes[2]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use GetPositionRequest.ProtoReflect.Descriptor instead.
func (*GetPositionRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_servo_v1_servo_proto_rawDescGZIP(), []int{2}
}

func (x *GetPositionRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

type GetPositionResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// the degrees from neutral by which the servo is currently rotated. Values are between 0 and 180
	PositionDeg uint32 `protobuf:"varint,1,opt,name=position_deg,json=positionDeg,proto3" json:"position_deg,omitempty"`
}

func (x *GetPositionResponse) Reset() {
	*x = GetPositionResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_servo_v1_servo_proto_msgTypes[3]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *GetPositionResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*GetPositionResponse) ProtoMessage() {}

func (x *GetPositionResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_servo_v1_servo_proto_msgTypes[3]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use GetPositionResponse.ProtoReflect.Descriptor instead.
func (*GetPositionResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_servo_v1_servo_proto_rawDescGZIP(), []int{3}
}

func (x *GetPositionResponse) GetPositionDeg() uint32 {
	if x != nil {
		return x.PositionDeg
	}
	return 0
}

var File_proto_api_component_servo_v1_servo_proto protoreflect.FileDescriptor

var file_proto_api_component_servo_v1_servo_proto_rawDesc = []byte{
	0x0a, 0x28, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x63, 0x6f, 0x6d, 0x70,
	0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x73, 0x65, 0x72, 0x76, 0x6f, 0x2f, 0x76, 0x31, 0x2f, 0x73,
	0x65, 0x72, 0x76, 0x6f, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x12, 0x1c, 0x70, 0x72, 0x6f, 0x74,
	0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e,
	0x73, 0x65, 0x72, 0x76, 0x6f, 0x2e, 0x76, 0x31, 0x1a, 0x1c, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65,
	0x2f, 0x61, 0x70, 0x69, 0x2f, 0x61, 0x6e, 0x6e, 0x6f, 0x74, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x73,
	0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x22, 0x3e, 0x0a, 0x0b, 0x4d, 0x6f, 0x76, 0x65, 0x52, 0x65,
	0x71, 0x75, 0x65, 0x73, 0x74, 0x12, 0x12, 0x0a, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x18, 0x01, 0x20,
	0x01, 0x28, 0x09, 0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x12, 0x1b, 0x0a, 0x09, 0x61, 0x6e, 0x67,
	0x6c, 0x65, 0x5f, 0x64, 0x65, 0x67, 0x18, 0x02, 0x20, 0x01, 0x28, 0x0d, 0x52, 0x08, 0x61, 0x6e,
	0x67, 0x6c, 0x65, 0x44, 0x65, 0x67, 0x22, 0x0e, 0x0a, 0x0c, 0x4d, 0x6f, 0x76, 0x65, 0x52, 0x65,
	0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22, 0x28, 0x0a, 0x12, 0x47, 0x65, 0x74, 0x50, 0x6f, 0x73,
	0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x12, 0x12, 0x0a, 0x04,
	0x6e, 0x61, 0x6d, 0x65, 0x18, 0x01, 0x20, 0x01, 0x28, 0x09, 0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65,
	0x22, 0x38, 0x0a, 0x13, 0x47, 0x65, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52,
	0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x12, 0x21, 0x0a, 0x0c, 0x70, 0x6f, 0x73, 0x69, 0x74,
	0x69, 0x6f, 0x6e, 0x5f, 0x64, 0x65, 0x67, 0x18, 0x01, 0x20, 0x01, 0x28, 0x0d, 0x52, 0x0b, 0x70,
	0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x44, 0x65, 0x67, 0x32, 0xc1, 0x02, 0x0a, 0x0c, 0x53,
	0x65, 0x72, 0x76, 0x6f, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x12, 0x8a, 0x01, 0x0a, 0x04,
	0x4d, 0x6f, 0x76, 0x65, 0x12, 0x29, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69,
	0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x73, 0x65, 0x72, 0x76, 0x6f,
	0x2e, 0x76, 0x31, 0x2e, 0x4d, 0x6f, 0x76, 0x65, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a,
	0x2a, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70,
	0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x73, 0x65, 0x72, 0x76, 0x6f, 0x2e, 0x76, 0x31, 0x2e, 0x4d,
	0x6f, 0x76, 0x65, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22, 0x2b, 0x82, 0xd3, 0xe4,
	0x93, 0x02, 0x25, 0x1a, 0x23, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x76, 0x31, 0x2f, 0x63, 0x6f, 0x6d,
	0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x73, 0x65, 0x72, 0x76, 0x6f, 0x2f, 0x7b, 0x6e, 0x61,
	0x6d, 0x65, 0x7d, 0x2f, 0x6d, 0x6f, 0x76, 0x65, 0x12, 0xa3, 0x01, 0x0a, 0x0b, 0x47, 0x65, 0x74,
	0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x12, 0x30, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f,
	0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x73,
	0x65, 0x72, 0x76, 0x6f, 0x2e, 0x76, 0x31, 0x2e, 0x47, 0x65, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74,
	0x69, 0x6f, 0x6e, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a, 0x31, 0x2e, 0x70, 0x72, 0x6f,
	0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74,
	0x2e, 0x73, 0x65, 0x72, 0x76, 0x6f, 0x2e, 0x76, 0x31, 0x2e, 0x47, 0x65, 0x74, 0x50, 0x6f, 0x73,
	0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22, 0x2f, 0x82,
	0xd3, 0xe4, 0x93, 0x02, 0x29, 0x12, 0x27, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x76, 0x31, 0x2f, 0x63,
	0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x73, 0x65, 0x72, 0x76, 0x6f, 0x2f, 0x7b,
	0x6e, 0x61, 0x6d, 0x65, 0x7d, 0x2f, 0x70, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x42, 0x4d,
	0x0a, 0x23, 0x63, 0x6f, 0x6d, 0x2e, 0x76, 0x69, 0x61, 0x6d, 0x2e, 0x72, 0x64, 0x6b, 0x2e, 0x70,
	0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65,
	0x6e, 0x74, 0x2e, 0x76, 0x31, 0x5a, 0x26, 0x67, 0x6f, 0x2e, 0x76, 0x69, 0x61, 0x6d, 0x2e, 0x63,
	0x6f, 0x6d, 0x2f, 0x72, 0x64, 0x6b, 0x2f, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x61, 0x70, 0x69,
	0x2f, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x76, 0x31, 0x62, 0x06, 0x70,
	0x72, 0x6f, 0x74, 0x6f, 0x33,
}

var (
	file_proto_api_component_servo_v1_servo_proto_rawDescOnce sync.Once
	file_proto_api_component_servo_v1_servo_proto_rawDescData = file_proto_api_component_servo_v1_servo_proto_rawDesc
)

func file_proto_api_component_servo_v1_servo_proto_rawDescGZIP() []byte {
	file_proto_api_component_servo_v1_servo_proto_rawDescOnce.Do(func() {
		file_proto_api_component_servo_v1_servo_proto_rawDescData = protoimpl.X.CompressGZIP(file_proto_api_component_servo_v1_servo_proto_rawDescData)
	})
	return file_proto_api_component_servo_v1_servo_proto_rawDescData
}

var file_proto_api_component_servo_v1_servo_proto_msgTypes = make([]protoimpl.MessageInfo, 4)
var file_proto_api_component_servo_v1_servo_proto_goTypes = []interface{}{
	(*MoveRequest)(nil),         // 0: proto.api.component.servo.v1.MoveRequest
	(*MoveResponse)(nil),        // 1: proto.api.component.servo.v1.MoveResponse
	(*GetPositionRequest)(nil),  // 2: proto.api.component.servo.v1.GetPositionRequest
	(*GetPositionResponse)(nil), // 3: proto.api.component.servo.v1.GetPositionResponse
}
var file_proto_api_component_servo_v1_servo_proto_depIdxs = []int32{
	0, // 0: proto.api.component.servo.v1.ServoService.Move:input_type -> proto.api.component.servo.v1.MoveRequest
	2, // 1: proto.api.component.servo.v1.ServoService.GetPosition:input_type -> proto.api.component.servo.v1.GetPositionRequest
	1, // 2: proto.api.component.servo.v1.ServoService.Move:output_type -> proto.api.component.servo.v1.MoveResponse
	3, // 3: proto.api.component.servo.v1.ServoService.GetPosition:output_type -> proto.api.component.servo.v1.GetPositionResponse
	2, // [2:4] is the sub-list for method output_type
	0, // [0:2] is the sub-list for method input_type
	0, // [0:0] is the sub-list for extension type_name
	0, // [0:0] is the sub-list for extension extendee
	0, // [0:0] is the sub-list for field type_name
}

func init() { file_proto_api_component_servo_v1_servo_proto_init() }
func file_proto_api_component_servo_v1_servo_proto_init() {
	if File_proto_api_component_servo_v1_servo_proto != nil {
		return
	}
	if !protoimpl.UnsafeEnabled {
		file_proto_api_component_servo_v1_servo_proto_msgTypes[0].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*MoveRequest); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_servo_v1_servo_proto_msgTypes[1].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*MoveResponse); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_servo_v1_servo_proto_msgTypes[2].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*GetPositionRequest); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_servo_v1_servo_proto_msgTypes[3].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*GetPositionResponse); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
	}
	type x struct{}
	out := protoimpl.TypeBuilder{
		File: protoimpl.DescBuilder{
			GoPackagePath: reflect.TypeOf(x{}).PkgPath(),
			RawDescriptor: file_proto_api_component_servo_v1_servo_proto_rawDesc,
			NumEnums:      0,
			NumMessages:   4,
			NumExtensions: 0,
			NumServices:   1,
		},
		GoTypes:           file_proto_api_component_servo_v1_servo_proto_goTypes,
		DependencyIndexes: file_proto_api_component_servo_v1_servo_proto_depIdxs,
		MessageInfos:      file_proto_api_component_servo_v1_servo_proto_msgTypes,
	}.Build()
	File_proto_api_component_servo_v1_servo_proto = out.File
	file_proto_api_component_servo_v1_servo_proto_rawDesc = nil
	file_proto_api_component_servo_v1_servo_proto_goTypes = nil
	file_proto_api_component_servo_v1_servo_proto_depIdxs = nil
}
