// Code generated by protoc-gen-go. DO NOT EDIT.
// versions:
// 	protoc-gen-go v1.27.1
// 	protoc        (unknown)
// source: proto/api/component/forcematrix/v1/force_matrix.proto

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

// Matrix
type Matrix struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Rows uint32   `protobuf:"varint,1,opt,name=rows,proto3" json:"rows,omitempty"`
	Cols uint32   `protobuf:"varint,2,opt,name=cols,proto3" json:"cols,omitempty"`
	Data []uint32 `protobuf:"varint,3,rep,packed,name=data,proto3" json:"data,omitempty"`
}

func (x *Matrix) Reset() {
	*x = Matrix{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[0]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *Matrix) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*Matrix) ProtoMessage() {}

func (x *Matrix) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[0]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use Matrix.ProtoReflect.Descriptor instead.
func (*Matrix) Descriptor() ([]byte, []int) {
	return file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescGZIP(), []int{0}
}

func (x *Matrix) GetRows() uint32 {
	if x != nil {
		return x.Rows
	}
	return 0
}

func (x *Matrix) GetCols() uint32 {
	if x != nil {
		return x.Cols
	}
	return 0
}

func (x *Matrix) GetData() []uint32 {
	if x != nil {
		return x.Data
	}
	return nil
}

// ForceMatrix
type ReadMatrixRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
}

func (x *ReadMatrixRequest) Reset() {
	*x = ReadMatrixRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[1]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ReadMatrixRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ReadMatrixRequest) ProtoMessage() {}

func (x *ReadMatrixRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[1]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ReadMatrixRequest.ProtoReflect.Descriptor instead.
func (*ReadMatrixRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescGZIP(), []int{1}
}

func (x *ReadMatrixRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

type ReadMatrixResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Matrix *Matrix `protobuf:"bytes,1,opt,name=matrix,proto3" json:"matrix,omitempty"`
}

func (x *ReadMatrixResponse) Reset() {
	*x = ReadMatrixResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[2]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ReadMatrixResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ReadMatrixResponse) ProtoMessage() {}

func (x *ReadMatrixResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[2]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ReadMatrixResponse.ProtoReflect.Descriptor instead.
func (*ReadMatrixResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescGZIP(), []int{2}
}

func (x *ReadMatrixResponse) GetMatrix() *Matrix {
	if x != nil {
		return x.Matrix
	}
	return nil
}

type DetectSlipRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
}

func (x *DetectSlipRequest) Reset() {
	*x = DetectSlipRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[3]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *DetectSlipRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*DetectSlipRequest) ProtoMessage() {}

func (x *DetectSlipRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[3]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use DetectSlipRequest.ProtoReflect.Descriptor instead.
func (*DetectSlipRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescGZIP(), []int{3}
}

func (x *DetectSlipRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

type DetectSlipResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	SlipDetected bool `protobuf:"varint,1,opt,name=slip_detected,json=slipDetected,proto3" json:"slip_detected,omitempty"`
}

func (x *DetectSlipResponse) Reset() {
	*x = DetectSlipResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[4]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *DetectSlipResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*DetectSlipResponse) ProtoMessage() {}

func (x *DetectSlipResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[4]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use DetectSlipResponse.ProtoReflect.Descriptor instead.
func (*DetectSlipResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescGZIP(), []int{4}
}

func (x *DetectSlipResponse) GetSlipDetected() bool {
	if x != nil {
		return x.SlipDetected
	}
	return false
}

var File_proto_api_component_forcematrix_v1_force_matrix_proto protoreflect.FileDescriptor

var file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDesc = []byte{
	0x0a, 0x35, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x63, 0x6f, 0x6d, 0x70,
	0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x66, 0x6f, 0x72, 0x63, 0x65, 0x6d, 0x61, 0x74, 0x72, 0x69,
	0x78, 0x2f, 0x76, 0x31, 0x2f, 0x66, 0x6f, 0x72, 0x63, 0x65, 0x5f, 0x6d, 0x61, 0x74, 0x72, 0x69,
	0x78, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x12, 0x22, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61,
	0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x66, 0x6f, 0x72,
	0x63, 0x65, 0x6d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x2e, 0x76, 0x31, 0x1a, 0x1c, 0x67, 0x6f, 0x6f,
	0x67, 0x6c, 0x65, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x61, 0x6e, 0x6e, 0x6f, 0x74, 0x61, 0x74, 0x69,
	0x6f, 0x6e, 0x73, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x22, 0x44, 0x0a, 0x06, 0x4d, 0x61, 0x74,
	0x72, 0x69, 0x78, 0x12, 0x12, 0x0a, 0x04, 0x72, 0x6f, 0x77, 0x73, 0x18, 0x01, 0x20, 0x01, 0x28,
	0x0d, 0x52, 0x04, 0x72, 0x6f, 0x77, 0x73, 0x12, 0x12, 0x0a, 0x04, 0x63, 0x6f, 0x6c, 0x73, 0x18,
	0x02, 0x20, 0x01, 0x28, 0x0d, 0x52, 0x04, 0x63, 0x6f, 0x6c, 0x73, 0x12, 0x12, 0x0a, 0x04, 0x64,
	0x61, 0x74, 0x61, 0x18, 0x03, 0x20, 0x03, 0x28, 0x0d, 0x52, 0x04, 0x64, 0x61, 0x74, 0x61, 0x22,
	0x27, 0x0a, 0x11, 0x52, 0x65, 0x61, 0x64, 0x4d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x52, 0x65, 0x71,
	0x75, 0x65, 0x73, 0x74, 0x12, 0x12, 0x0a, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x18, 0x01, 0x20, 0x01,
	0x28, 0x09, 0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x22, 0x58, 0x0a, 0x12, 0x52, 0x65, 0x61, 0x64,
	0x4d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x12, 0x42,
	0x0a, 0x06, 0x6d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x18, 0x01, 0x20, 0x01, 0x28, 0x0b, 0x32, 0x2a,
	0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f,
	0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x66, 0x6f, 0x72, 0x63, 0x65, 0x6d, 0x61, 0x74, 0x72, 0x69, 0x78,
	0x2e, 0x76, 0x31, 0x2e, 0x4d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x52, 0x06, 0x6d, 0x61, 0x74, 0x72,
	0x69, 0x78, 0x22, 0x27, 0x0a, 0x11, 0x44, 0x65, 0x74, 0x65, 0x63, 0x74, 0x53, 0x6c, 0x69, 0x70,
	0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x12, 0x12, 0x0a, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x18,
	0x01, 0x20, 0x01, 0x28, 0x09, 0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x22, 0x39, 0x0a, 0x12, 0x44,
	0x65, 0x74, 0x65, 0x63, 0x74, 0x53, 0x6c, 0x69, 0x70, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73,
	0x65, 0x12, 0x23, 0x0a, 0x0d, 0x73, 0x6c, 0x69, 0x70, 0x5f, 0x64, 0x65, 0x74, 0x65, 0x63, 0x74,
	0x65, 0x64, 0x18, 0x01, 0x20, 0x01, 0x28, 0x08, 0x52, 0x0c, 0x73, 0x6c, 0x69, 0x70, 0x44, 0x65,
	0x74, 0x65, 0x63, 0x74, 0x65, 0x64, 0x32, 0x84, 0x03, 0x0a, 0x12, 0x46, 0x6f, 0x72, 0x63, 0x65,
	0x4d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x12, 0xb1, 0x01,
	0x0a, 0x0a, 0x52, 0x65, 0x61, 0x64, 0x4d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x12, 0x35, 0x2e, 0x70,
	0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65,
	0x6e, 0x74, 0x2e, 0x66, 0x6f, 0x72, 0x63, 0x65, 0x6d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x2e, 0x76,
	0x31, 0x2e, 0x52, 0x65, 0x61, 0x64, 0x4d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x52, 0x65, 0x71, 0x75,
	0x65, 0x73, 0x74, 0x1a, 0x36, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e,
	0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x66, 0x6f, 0x72, 0x63, 0x65, 0x6d,
	0x61, 0x74, 0x72, 0x69, 0x78, 0x2e, 0x76, 0x31, 0x2e, 0x52, 0x65, 0x61, 0x64, 0x4d, 0x61, 0x74,
	0x72, 0x69, 0x78, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22, 0x34, 0x82, 0xd3, 0xe4,
	0x93, 0x02, 0x2e, 0x12, 0x2c, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x76, 0x31, 0x2f, 0x63, 0x6f, 0x6d,
	0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x66, 0x6f, 0x72, 0x63, 0x65, 0x5f, 0x6d, 0x61, 0x74,
	0x72, 0x69, 0x78, 0x2f, 0x7b, 0x6e, 0x61, 0x6d, 0x65, 0x7d, 0x2f, 0x6d, 0x61, 0x74, 0x72, 0x69,
	0x78, 0x12, 0xb9, 0x01, 0x0a, 0x0a, 0x44, 0x65, 0x74, 0x65, 0x63, 0x74, 0x53, 0x6c, 0x69, 0x70,
	0x12, 0x35, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d,
	0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x66, 0x6f, 0x72, 0x63, 0x65, 0x6d, 0x61, 0x74, 0x72,
	0x69, 0x78, 0x2e, 0x76, 0x31, 0x2e, 0x44, 0x65, 0x74, 0x65, 0x63, 0x74, 0x53, 0x6c, 0x69, 0x70,
	0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a, 0x36, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e,
	0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x66, 0x6f,
	0x72, 0x63, 0x65, 0x6d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x2e, 0x76, 0x31, 0x2e, 0x44, 0x65, 0x74,
	0x65, 0x63, 0x74, 0x53, 0x6c, 0x69, 0x70, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22,
	0x3c, 0x82, 0xd3, 0xe4, 0x93, 0x02, 0x36, 0x12, 0x34, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x76, 0x31,
	0x2f, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x66, 0x6f, 0x72, 0x63, 0x65,
	0x5f, 0x6d, 0x61, 0x74, 0x72, 0x69, 0x78, 0x2f, 0x7b, 0x6e, 0x61, 0x6d, 0x65, 0x7d, 0x2f, 0x73,
	0x6c, 0x69, 0x70, 0x5f, 0x64, 0x65, 0x74, 0x65, 0x63, 0x74, 0x69, 0x6f, 0x6e, 0x42, 0x4d, 0x0a,
	0x23, 0x63, 0x6f, 0x6d, 0x2e, 0x76, 0x69, 0x61, 0x6d, 0x2e, 0x72, 0x64, 0x6b, 0x2e, 0x70, 0x72,
	0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e,
	0x74, 0x2e, 0x76, 0x31, 0x5a, 0x26, 0x67, 0x6f, 0x2e, 0x76, 0x69, 0x61, 0x6d, 0x2e, 0x63, 0x6f,
	0x6d, 0x2f, 0x72, 0x64, 0x6b, 0x2f, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x61, 0x70, 0x69, 0x2f,
	0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x76, 0x31, 0x62, 0x06, 0x70, 0x72,
	0x6f, 0x74, 0x6f, 0x33,
}

var (
	file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescOnce sync.Once
	file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescData = file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDesc
)

func file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescGZIP() []byte {
	file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescOnce.Do(func() {
		file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescData = protoimpl.X.CompressGZIP(file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescData)
	})
	return file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDescData
}

var file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes = make([]protoimpl.MessageInfo, 5)
var file_proto_api_component_forcematrix_v1_force_matrix_proto_goTypes = []interface{}{
	(*Matrix)(nil),             // 0: proto.api.component.forcematrix.v1.Matrix
	(*ReadMatrixRequest)(nil),  // 1: proto.api.component.forcematrix.v1.ReadMatrixRequest
	(*ReadMatrixResponse)(nil), // 2: proto.api.component.forcematrix.v1.ReadMatrixResponse
	(*DetectSlipRequest)(nil),  // 3: proto.api.component.forcematrix.v1.DetectSlipRequest
	(*DetectSlipResponse)(nil), // 4: proto.api.component.forcematrix.v1.DetectSlipResponse
}
var file_proto_api_component_forcematrix_v1_force_matrix_proto_depIdxs = []int32{
	0, // 0: proto.api.component.forcematrix.v1.ReadMatrixResponse.matrix:type_name -> proto.api.component.forcematrix.v1.Matrix
	1, // 1: proto.api.component.forcematrix.v1.ForceMatrixService.ReadMatrix:input_type -> proto.api.component.forcematrix.v1.ReadMatrixRequest
	3, // 2: proto.api.component.forcematrix.v1.ForceMatrixService.DetectSlip:input_type -> proto.api.component.forcematrix.v1.DetectSlipRequest
	2, // 3: proto.api.component.forcematrix.v1.ForceMatrixService.ReadMatrix:output_type -> proto.api.component.forcematrix.v1.ReadMatrixResponse
	4, // 4: proto.api.component.forcematrix.v1.ForceMatrixService.DetectSlip:output_type -> proto.api.component.forcematrix.v1.DetectSlipResponse
	3, // [3:5] is the sub-list for method output_type
	1, // [1:3] is the sub-list for method input_type
	1, // [1:1] is the sub-list for extension type_name
	1, // [1:1] is the sub-list for extension extendee
	0, // [0:1] is the sub-list for field type_name
}

func init() { file_proto_api_component_forcematrix_v1_force_matrix_proto_init() }
func file_proto_api_component_forcematrix_v1_force_matrix_proto_init() {
	if File_proto_api_component_forcematrix_v1_force_matrix_proto != nil {
		return
	}
	if !protoimpl.UnsafeEnabled {
		file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[0].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*Matrix); i {
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
		file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[1].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ReadMatrixRequest); i {
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
		file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[2].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ReadMatrixResponse); i {
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
		file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[3].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*DetectSlipRequest); i {
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
		file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes[4].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*DetectSlipResponse); i {
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
			RawDescriptor: file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDesc,
			NumEnums:      0,
			NumMessages:   5,
			NumExtensions: 0,
			NumServices:   1,
		},
		GoTypes:           file_proto_api_component_forcematrix_v1_force_matrix_proto_goTypes,
		DependencyIndexes: file_proto_api_component_forcematrix_v1_force_matrix_proto_depIdxs,
		MessageInfos:      file_proto_api_component_forcematrix_v1_force_matrix_proto_msgTypes,
	}.Build()
	File_proto_api_component_forcematrix_v1_force_matrix_proto = out.File
	file_proto_api_component_forcematrix_v1_force_matrix_proto_rawDesc = nil
	file_proto_api_component_forcematrix_v1_force_matrix_proto_goTypes = nil
	file_proto_api_component_forcematrix_v1_force_matrix_proto_depIdxs = nil
}
