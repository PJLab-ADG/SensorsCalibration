// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2018 Rasmus Munk Larsen <rmlarsen@google.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_TYPE_CASTING_NEON_H
#define EIGEN_TYPE_CASTING_NEON_H

namespace Eigen {

namespace internal {

template<> struct type_casting_traits<float,numext::int8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<float,numext::uint8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<float,numext::int16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<float,numext::uint16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<float,numext::int32_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<float,numext::uint32_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<float,numext::int64_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<float,numext::uint64_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int8_t,float>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int8_t,numext::uint8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int8_t,numext::int16_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int8_t,numext::uint16_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int8_t,numext::int32_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int8_t,numext::uint32_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint8_t,float>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint8_t,numext::int8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint8_t,numext::int16_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint8_t,numext::uint16_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint8_t,numext::int32_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint8_t,numext::uint32_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int16_t,float>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int16_t,numext::int8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int16_t,numext::uint8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int16_t,numext::uint16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int16_t,numext::int32_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int16_t,numext::uint32_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int16_t,numext::int64_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int16_t,numext::uint64_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint16_t,float>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint16_t,numext::int8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint16_t,numext::uint8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint16_t,numext::int16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint16_t,numext::int32_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint16_t,numext::uint32_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint16_t,numext::int64_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint16_t,numext::uint64_t>
{ enum { VectorizedCast = 0, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int32_t,float>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int32_t,numext::int8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int32_t,numext::uint8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int32_t,numext::int16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int32_t,numext::uint16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int32_t,numext::uint32_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int32_t,numext::int64_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int32_t,numext::uint64_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint32_t,float>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint32_t,numext::int8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint32_t,numext::uint8_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint32_t,numext::int16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint32_t,numext::uint16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint32_t,numext::int32_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint32_t,numext::int64_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint32_t,numext::uint64_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int64_t,float>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int64_t,numext::int16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int64_t,numext::uint16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int64_t,numext::uint32_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::int64_t,numext::uint64_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint64_t,float>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint64_t,numext::int16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint64_t,numext::uint16_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint64_t,numext::int32_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };
template<> struct type_casting_traits<numext::uint64_t,numext::int64_t>
{ enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 }; };

template<> EIGEN_STRONG_INLINE Packet2f pcast<Packet2i,Packet2f>(const Packet2i& a) { return vcvt_f32_s32(a); }
template<> EIGEN_STRONG_INLINE Packet2f pcast<Packet2ui,Packet2f>(const Packet2ui& a) { return vcvt_f32_u32(a); }
template<> EIGEN_STRONG_INLINE Packet2f pcast<Packet2l,Packet2f>(const Packet2l& a)
{ return vcvt_f32_s32(vmovn_s64(a)); }
template<> EIGEN_STRONG_INLINE Packet2f pcast<Packet2ul,Packet2f>(const Packet2ul& a)
{ return vcvt_f32_u32(vmovn_u64(a)); }
template<> EIGEN_STRONG_INLINE Packet4f pcast<Packet4c,Packet4f>(const Packet4c& a)
{ return vcvtq_f32_s32(vmovl_s16(vget_low_s16(vmovl_s8(vreinterpret_s8_s32(vdup_n_s32(a)))))); }
template<> EIGEN_STRONG_INLINE Packet4f pcast<Packet4uc,Packet4f>(const Packet4uc& a)
{ return vcvtq_f32_s32(vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(vmovl_u8(vreinterpret_u8_u32(vdup_n_u32(a))))))); }
template<> EIGEN_STRONG_INLINE Packet4f pcast<Packet4s,Packet4f>(const Packet4s& a)
{ return vcvtq_f32_s32(vmovl_s16(a)); }
template<> EIGEN_STRONG_INLINE Packet4f pcast<Packet4us,Packet4f>(const Packet4us& a)
{ return vcvtq_f32_s32(vreinterpretq_s32_u32(vmovl_u16(a))); }
template<> EIGEN_STRONG_INLINE Packet4f pcast<Packet4i,Packet4f>(const Packet4i& a) { return vcvtq_f32_s32(a); }
template<> EIGEN_STRONG_INLINE Packet4f pcast<Packet4ui,Packet4f>(const Packet4ui& a) { return vcvtq_f32_u32(a); }
template<> EIGEN_STRONG_INLINE Packet4c pcast<Packet4f,Packet4c>(const Packet4f& a)
{
  const int16x4_t b = vmovn_s32(vcvtq_s32_f32(a));
  return vget_lane_s32(vreinterpret_s32_s8(vmovn_s16(vcombine_s16(b, b))), 0);
}
template<> EIGEN_STRONG_INLINE Packet4c pcast<Packet4uc,Packet4c>(const Packet4uc& a)
{ return static_cast<Packet4c>(a); }
template<> EIGEN_STRONG_INLINE Packet4c pcast<Packet4s,Packet4c>(const Packet4s& a)
{ return vget_lane_s32(vreinterpret_s32_s8(vmovn_s16(vcombine_s16(a, a))), 0); }
template<> EIGEN_STRONG_INLINE Packet4c pcast<Packet4us,Packet4c>(const Packet4us& a)
{
  const int16x4_t b = vreinterpret_s16_u16(a);
  return vget_lane_s32(vreinterpret_s32_s8(vmovn_s16(vcombine_s16(b, b))), 0);
}
template<> EIGEN_STRONG_INLINE Packet4c pcast<Packet4i,Packet4c>(const Packet4i& a)
{
  const int16x4_t b = vmovn_s32(a);
  return vget_lane_s32(vreinterpret_s32_s8(vmovn_s16(vcombine_s16(b, b))), 0);
}
template<> EIGEN_STRONG_INLINE Packet4c pcast<Packet4ui,Packet4c>(const Packet4ui& a)
{
  const int16x4_t b = vmovn_s32(vreinterpretq_s32_u32(a));
  return vget_lane_s32(vreinterpret_s32_s8(vmovn_s16(vcombine_s16(b, b))), 0);
}
template<> EIGEN_STRONG_INLINE Packet8c pcast<Packet8uc,Packet8c>(const Packet8uc& a) { return vreinterpret_s8_u8(a); }
template<> EIGEN_STRONG_INLINE Packet8c pcast<Packet8s,Packet8c>(const Packet8s& a) { return vmovn_s16(a); }
template<> EIGEN_STRONG_INLINE Packet8c pcast<Packet8us,Packet8c>(const Packet8us& a)
{ return vreinterpret_s8_u8(vmovn_u16(a)); }
template<> EIGEN_STRONG_INLINE Packet16c pcast<Packet16uc,Packet16c>(const Packet16uc& a)
{ return vreinterpretq_s8_u8(a); }
template<> EIGEN_STRONG_INLINE Packet4uc pcast<Packet4f,Packet4uc>(const Packet4f& a)
{
  const uint16x4_t b = vmovn_u32(vreinterpretq_u32_s32(vcvtq_s32_f32(a)));
  return vget_lane_u32(vreinterpret_u32_u8(vmovn_u16(vcombine_u16(b, b))), 0);
}
template<> EIGEN_STRONG_INLINE Packet4uc pcast<Packet4i,Packet4uc>(const Packet4i& a)
{
  const uint16x4_t b = vmovn_u32(vreinterpretq_u32_s32(a));
  return vget_lane_u32(vreinterpret_u32_u8(vmovn_u16(vcombine_u16(b, b))), 0);
}
template<> EIGEN_STRONG_INLINE Packet4uc pcast<Packet4ui,Packet4uc>(const Packet4ui& a)
{
  const uint16x4_t b = vmovn_u32(a);
  return vget_lane_u32(vreinterpret_u32_u8(vmovn_u16(vcombine_u16(b, b))), 0);
}
template<> EIGEN_STRONG_INLINE Packet4uc pcast<Packet4c,Packet4uc>(const Packet4c& a)
{ return static_cast<Packet4uc>(a); }
template<> EIGEN_STRONG_INLINE Packet4uc pcast<Packet4s,Packet4uc>(const Packet4s& a)
{
  const uint16x4_t b = vreinterpret_u16_s16(a);
  return vget_lane_u32(vreinterpret_u32_u8(vmovn_u16(vcombine_u16(b, b))), 0);
}
template<> EIGEN_STRONG_INLINE Packet4uc pcast<Packet4us,Packet4uc>(const Packet4us& a)
{ return vget_lane_u32(vreinterpret_u32_u8(vmovn_u16(vcombine_u16(a, a))), 0); }
template<> EIGEN_STRONG_INLINE Packet8uc pcast<Packet8c,Packet8uc>(const Packet8c& a) { return vreinterpret_u8_s8(a); }
template<> EIGEN_STRONG_INLINE Packet8uc pcast<Packet8s,Packet8uc>(const Packet8s& a)
{ return vreinterpret_u8_s8(vmovn_s16(a)); }
template<> EIGEN_STRONG_INLINE Packet8uc pcast<Packet8us,Packet8uc>(const Packet8us& a) { return vmovn_u16(a); }
template<> EIGEN_STRONG_INLINE Packet16uc pcast<Packet16c,Packet16uc>(const Packet16c& a)
{ return vreinterpretq_u8_s8(a); }
template<> EIGEN_STRONG_INLINE Packet4s pcast<Packet4f,Packet4s>(const Packet4f& a)
{ return vmovn_s32(vcvtq_s32_f32(a)); }
template<> EIGEN_STRONG_INLINE Packet4s pcast<Packet4c,Packet4s>(const Packet4c& a)
{ return vget_low_s16(vmovl_s8(vreinterpret_s8_s32(vdup_n_s32(a)))); }
template<> EIGEN_STRONG_INLINE Packet4s pcast<Packet4uc,Packet4s>(const Packet4uc& a)
{ return vget_low_s16(vreinterpretq_s16_u16(vmovl_u8(vreinterpret_u8_u32(vdup_n_u32(a))))); }
template<> EIGEN_STRONG_INLINE Packet4s pcast<Packet4us,Packet4s>(const Packet4us& a)
{ return vreinterpret_s16_u16(a); }
template<> EIGEN_STRONG_INLINE Packet4s pcast<Packet4i,Packet4s>(const Packet4i& a) { return vmovn_s32(a); }
template<> EIGEN_STRONG_INLINE Packet4s pcast<Packet4ui,Packet4s>(const Packet4ui& a)
{ return vmovn_s32(vreinterpretq_s32_u32(a)); }
template<> EIGEN_STRONG_INLINE Packet8s pcast<Packet8uc,Packet8s>(const Packet8uc& a)
{ return vreinterpretq_s16_u16(vmovl_u8(a)); }
template<> EIGEN_STRONG_INLINE Packet8s pcast<Packet8c,Packet8s>(const Packet8c& a) { return vmovl_s8(a); }
template<> EIGEN_STRONG_INLINE Packet8s pcast<Packet8us,Packet8s>(const Packet8us& a)
{ return vreinterpretq_s16_u16(a); }
template<> EIGEN_STRONG_INLINE Packet4us pcast<Packet4f,Packet4us>(const Packet4f& a)
{ return vmovn_u32(vreinterpretq_u32_s32(vcvtq_s32_f32(a))); }
template<> EIGEN_STRONG_INLINE Packet4us pcast<Packet4c,Packet4us>(const Packet4c& a)
{ return vget_low_u16(vreinterpretq_u16_s16(vmovl_s8(vreinterpret_s8_s32(vdup_n_s32(a))))); }
template<> EIGEN_STRONG_INLINE Packet4us pcast<Packet4uc,Packet4us>(const Packet4uc& a)
{ return vget_low_u16(vmovl_u8(vreinterpret_u8_u32(vdup_n_u32(a)))); }
template<> EIGEN_STRONG_INLINE Packet4us pcast<Packet4s,Packet4us>(const Packet4s& a)
{ return vreinterpret_u16_s16(a); }
template<> EIGEN_STRONG_INLINE Packet4us pcast<Packet4i,Packet4us>(const Packet4i& a)
{ return vmovn_u32(vreinterpretq_u32_s32(a)); }
template<> EIGEN_STRONG_INLINE Packet4us pcast<Packet4ui,Packet4us>(const Packet4ui& a) { return vmovn_u32(a); }
template<> EIGEN_STRONG_INLINE Packet8us pcast<Packet8c,Packet8us>(const Packet8c& a)
{ return vreinterpretq_u16_s16(vmovl_s8(a)); }
template<> EIGEN_STRONG_INLINE Packet8us pcast<Packet8uc,Packet8us>(const Packet8uc& a) { return vmovl_u8(a); }
template<> EIGEN_STRONG_INLINE Packet8us pcast<Packet8s,Packet8us>(const Packet8s& a)
{ return vreinterpretq_u16_s16(a); }
template<> EIGEN_STRONG_INLINE Packet2i pcast<Packet2f,Packet2i>(const Packet2f& a) { return vcvt_s32_f32(a); }
template<> EIGEN_STRONG_INLINE Packet2i pcast<Packet2ui,Packet2i>(const Packet2ui& a)
{ return vreinterpret_s32_u32(a); }
template<> EIGEN_STRONG_INLINE Packet2i pcast<Packet2l,Packet2i>(const Packet2l& a)
{ return vmovn_s64(a); }
template<> EIGEN_STRONG_INLINE Packet2i pcast<Packet2ul,Packet2i>(const Packet2ul& a)
{ return vmovn_s64(vreinterpretq_s64_u64(a)); }
template<> EIGEN_STRONG_INLINE Packet4i pcast<Packet4f,Packet4i>(const Packet4f& a) { return vcvtq_s32_f32(a); }
template<> EIGEN_STRONG_INLINE Packet4i pcast<Packet4c,Packet4i>(const Packet4c& a)
{ return vmovl_s16(vget_low_s16(vmovl_s8(vreinterpret_s8_s32(vdup_n_s32(a))))); }
template<> EIGEN_STRONG_INLINE Packet4i pcast<Packet4uc,Packet4i>(const Packet4uc& a)
{ return vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(vmovl_u8(vreinterpret_u8_u32(vdup_n_u32(a)))))); }
template<> EIGEN_STRONG_INLINE Packet4i pcast<Packet4s,Packet4i>(const Packet4s& a) { return vmovl_s16(a); }
template<> EIGEN_STRONG_INLINE Packet4i pcast<Packet4us,Packet4i>(const Packet4us& a)
{ return vreinterpretq_s32_u32(vmovl_u16(a)); }
template<> EIGEN_STRONG_INLINE Packet4i pcast<Packet4ui,Packet4i>(const Packet4ui& a)
{ return vreinterpretq_s32_u32(a); }
template<> EIGEN_STRONG_INLINE Packet2ui pcast<Packet2f,Packet2ui>(const Packet2f& a) { return vcvt_u32_f32(a); }
template<> EIGEN_STRONG_INLINE Packet2ui pcast<Packet2i,Packet2ui>(const Packet2i& a)
{ return vreinterpret_u32_s32(a); }
template<> EIGEN_STRONG_INLINE Packet2ui pcast<Packet2l,Packet2ui>(const Packet2l& a)
{ return vmovn_u64(vreinterpretq_u64_s64(a)); }
template<> EIGEN_STRONG_INLINE Packet2ui pcast<Packet2ul,Packet2ui>(const Packet2ul& a)
{ return vmovn_u64(a); }
template<> EIGEN_STRONG_INLINE Packet4ui pcast<Packet4f,Packet4ui>(const Packet4f& a) { return vcvtq_u32_f32(a); }
template<> EIGEN_STRONG_INLINE Packet4ui pcast<Packet4c,Packet4ui>(const Packet4c& a)
{ return vreinterpretq_u32_s32(vmovl_s16(vget_low_s16(vmovl_s8(vreinterpret_s8_s32(vdup_n_s32(a)))))); }
template<> EIGEN_STRONG_INLINE Packet4ui pcast<Packet4uc,Packet4ui>(const Packet4uc& a)
{ return vmovl_u16(vget_low_u16(vmovl_u8(vreinterpret_u8_u32(vdup_n_u32(a))))); }
template<> EIGEN_STRONG_INLINE Packet4ui pcast<Packet4s,Packet4ui>(const Packet4s& a)
{ return vreinterpretq_u32_s32(vmovl_s16(a)); }
template<> EIGEN_STRONG_INLINE Packet4ui pcast<Packet4us,Packet4ui>(const Packet4us& a) { return vmovl_u16(a); }
template<> EIGEN_STRONG_INLINE Packet4ui pcast<Packet4i,Packet4ui>(const Packet4i& a)
{ return vreinterpretq_u32_s32(a); }
template<> EIGEN_STRONG_INLINE Packet2l pcast<Packet2f,Packet2l>(const Packet2f& a)
{ return vmovl_s32(vcvt_s32_f32(a)); }
template<> EIGEN_STRONG_INLINE Packet2l pcast<Packet2i,Packet2l>(const Packet2i& a)
{ return vmovl_s32(a); }
template<> EIGEN_STRONG_INLINE Packet2l pcast<Packet2ui,Packet2l>(const Packet2ui& a)
{ return vreinterpretq_s64_u64(vmovl_u32(a)); }
template<> EIGEN_STRONG_INLINE Packet2l pcast<Packet2ul,Packet2l>(const Packet2ul& a)
{ return vreinterpretq_s64_u64(a); }
template<> EIGEN_STRONG_INLINE Packet2ul pcast<Packet2f,Packet2ul>(const Packet2f& a)
{ return vmovl_u32(vcvt_u32_f32(a)); }
template<> EIGEN_STRONG_INLINE Packet2ul pcast<Packet2i,Packet2ul>(const Packet2i& a)
{ return vreinterpretq_u64_s64(vmovl_s32(a)); }
template<> EIGEN_STRONG_INLINE Packet2ul pcast<Packet2ui,Packet2ul>(const Packet2ui& a)
{ return vmovl_u32(a); }
template<> EIGEN_STRONG_INLINE Packet2ul pcast<Packet2l,Packet2ul>(const Packet2l& a)
{ return vreinterpretq_u64_s64(a); }

template<> EIGEN_STRONG_INLINE Packet2f preinterpret<Packet2f,Packet2i>(const Packet2i& a)
{ return vreinterpret_f32_s32(a); }
template<> EIGEN_STRONG_INLINE Packet2f preinterpret<Packet2f,Packet2ui>(const Packet2ui& a)
{ return vreinterpret_f32_u32(a); }
template<> EIGEN_STRONG_INLINE Packet4f preinterpret<Packet4f,Packet4i>(const Packet4i& a)
{ return vreinterpretq_f32_s32(a); }
template<> EIGEN_STRONG_INLINE Packet4f preinterpret<Packet4f,Packet4ui>(const Packet4ui& a)
{ return vreinterpretq_f32_u32(a); }
template<> EIGEN_STRONG_INLINE Packet4c preinterpret<Packet4c,Packet4uc>(const Packet4uc& a)
{ return static_cast<Packet4c>(a); }
template<> EIGEN_STRONG_INLINE Packet8c preinterpret<Packet8c,Packet8uc>(const Packet8uc& a)
{ return vreinterpret_s8_u8(a); }
template<> EIGEN_STRONG_INLINE Packet16c preinterpret<Packet16c,Packet16uc>(const Packet16uc& a)
{ return vreinterpretq_s8_u8(a); }
template<> EIGEN_STRONG_INLINE Packet4uc preinterpret<Packet4uc,Packet4c>(const Packet4c& a)
{ return static_cast<Packet4uc>(a); }
template<> EIGEN_STRONG_INLINE Packet8uc preinterpret<Packet8uc,Packet8c>(const Packet8c& a)
{ return vreinterpret_u8_s8(a); }
template<> EIGEN_STRONG_INLINE Packet16uc preinterpret<Packet16uc,Packet16c>(const Packet16c& a)
{ return vreinterpretq_u8_s8(a); }
template<> EIGEN_STRONG_INLINE Packet4s preinterpret<Packet4s,Packet4us>(const Packet4us& a)
{ return vreinterpret_s16_u16(a); }
template<> EIGEN_STRONG_INLINE Packet8s preinterpret<Packet8s,Packet8us>(const Packet8us& a)
{ return vreinterpretq_s16_u16(a); }
template<> EIGEN_STRONG_INLINE Packet4us preinterpret<Packet4us,Packet4s>(const Packet4s& a)
{ return vreinterpret_u16_s16(a); }
template<> EIGEN_STRONG_INLINE Packet8us preinterpret<Packet8us,Packet8s>(const Packet8s& a)
{ return vreinterpretq_u16_s16(a); }
template<> EIGEN_STRONG_INLINE Packet2i preinterpret<Packet2i,Packet2f>(const Packet2f& a)
{ return vreinterpret_s32_f32(a); }
template<> EIGEN_STRONG_INLINE Packet2i preinterpret<Packet2i,Packet2ui>(const Packet2ui& a)
{ return vreinterpret_s32_u32(a); }
template<> EIGEN_STRONG_INLINE Packet4i preinterpret<Packet4i,Packet4f>(const Packet4f& a)
{ return vreinterpretq_s32_f32(a); }
template<> EIGEN_STRONG_INLINE Packet4i preinterpret<Packet4i,Packet4ui>(const Packet4ui& a)
{ return vreinterpretq_s32_u32(a); }
template<> EIGEN_STRONG_INLINE Packet2ui preinterpret<Packet2ui,Packet2f>(const Packet2f& a)
{ return vreinterpret_u32_f32(a); }
template<> EIGEN_STRONG_INLINE Packet2ui preinterpret<Packet2ui,Packet2i>(const Packet2i& a)
{ return vreinterpret_u32_s32(a); }
template<> EIGEN_STRONG_INLINE Packet4ui preinterpret<Packet4ui,Packet4f>(const Packet4f& a)
{ return vreinterpretq_u32_f32(a); }
template<> EIGEN_STRONG_INLINE Packet4ui preinterpret<Packet4ui,Packet4i>(const Packet4i& a)
{ return vreinterpretq_u32_s32(a); }
template<> EIGEN_STRONG_INLINE Packet2l preinterpret<Packet2l,Packet2ul>(const Packet2ul& a)
{ return vreinterpretq_s64_u64(a); }
template<> EIGEN_STRONG_INLINE Packet2ul preinterpret<Packet2ul,Packet2l>(const Packet2l& a)
{ return vreinterpretq_u64_s64(a); }

#if EIGEN_ARCH_ARM64

template<> EIGEN_STRONG_INLINE Packet2f pcast<Packet2d,Packet2f>(const Packet2d& a) { return vcvt_f32_f64(a); }
template<> EIGEN_STRONG_INLINE Packet2d pcast<Packet2f,Packet2d>(const Packet2f& a) { return vcvt_f64_f32(a); }
template<> EIGEN_STRONG_INLINE Packet2d pcast<Packet2i,Packet2d>(const Packet2i& a) { return vcvtq_f64_s64(vmovl_s32(a)); }
template<> EIGEN_STRONG_INLINE Packet2d pcast<Packet2ui,Packet2d>(const Packet2ui& a) { return vcvtq_f64_u64(vmovl_u32(a)); }
template<> EIGEN_STRONG_INLINE Packet2d pcast<Packet2l,Packet2d>(const Packet2l& a) { return vcvtq_f64_s64(a); }
template<> EIGEN_STRONG_INLINE Packet2d pcast<Packet2ul,Packet2d>(const Packet2ul& a) { return vcvtq_f64_u64(a); }
template<> EIGEN_STRONG_INLINE Packet2i pcast<Packet2d,Packet2i>(const Packet2d& a) { return vcvt_s32_f32(vcvt_f32_f64(a)); }
template<> EIGEN_STRONG_INLINE Packet2ui pcast<Packet2d,Packet2ui>(const Packet2d& a) { return vcvt_u32_f32(vcvt_f32_f64(a)); }
template<> EIGEN_STRONG_INLINE Packet2l pcast<Packet2d,Packet2l>(const Packet2d& a) { return vcvtq_s64_f64(a); }
template<> EIGEN_STRONG_INLINE Packet2ul pcast<Packet2d,Packet2ul>(const Packet2d& a) { return vcvtq_u64_f64(a); }

template<> EIGEN_STRONG_INLINE Packet2d preinterpret<Packet2d,Packet2l>(const Packet2l& a)
{ return vreinterpretq_f64_s64(a); }
template<> EIGEN_STRONG_INLINE Packet2d preinterpret<Packet2d,Packet2ul>(const Packet2ul& a)
{ return vreinterpretq_f64_u64(a); }
template<> EIGEN_STRONG_INLINE Packet2l preinterpret<Packet2l,Packet2d>(const Packet2d& a)
{ return vreinterpretq_s64_f64(a); }
template<> EIGEN_STRONG_INLINE Packet2ul preinterpret<Packet2ul,Packet2d>(const Packet2d& a)
{ return vreinterpretq_u64_f64(a); }

#endif // EIGEN_ARCH_ARM64

} // end namespace internal

} // end namespace Eigen

#endif // EIGEN_TYPE_CASTING_NEON_H
