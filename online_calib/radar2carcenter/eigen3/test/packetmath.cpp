// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "packetmath_test_shared.h"

#define REF_ADD(a,b) ((a)+(b))
#define REF_SUB(a,b) ((a)-(b))
#define REF_MUL(a,b) ((a)*(b))
#define REF_DIV(a,b) ((a)/(b))
#define REF_ABS_DIFF(a,b) ((a)>(b)?(a)-(b):(b)-(a))

template<typename FromScalar, typename FromPacket, typename ToScalar, typename ToPacket, bool CanCast = false>
struct test_cast_helper;

template<typename FromScalar, typename FromPacket, typename ToScalar, typename ToPacket>
struct test_cast_helper<FromScalar, FromPacket, ToScalar, ToPacket, false> {
  static void run() {}
};

template<typename FromScalar, typename FromPacket, typename ToScalar, typename ToPacket>
struct test_cast_helper<FromScalar, FromPacket, ToScalar, ToPacket, true> {
  static void run() {
    static const int PacketSize = internal::unpacket_traits<FromPacket>::size;
    EIGEN_ALIGN_MAX FromScalar data1[PacketSize];
    EIGEN_ALIGN_MAX ToScalar data2[PacketSize];
    EIGEN_ALIGN_MAX ToScalar ref[PacketSize];

    // Construct a packet of scalars that will not overflow when casting
    for (int i=0; i<PacketSize; ++i) {
      const FromScalar from_scalar = Array<FromScalar,1,1>::Random().value();
      const ToScalar to_scalar = Array<ToScalar,1,1>::Random().value();
      const FromScalar c = sizeof(ToScalar) > sizeof(FromScalar) ? static_cast<FromScalar>(to_scalar) : from_scalar;
      data1[i] = (NumTraits<FromScalar>::IsSigned && !NumTraits<ToScalar>::IsSigned) ? numext::abs(c) : c;
    }

    for (int i=0; i<PacketSize; ++i)
      ref[i] = static_cast<const ToScalar>(data1[i]);
    internal::pstore(data2, internal::pcast<FromPacket, ToPacket>(internal::pload<FromPacket>(data1)));

    VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::pcast<>");
  }
};

template<typename FromPacket, typename ToScalar>
void test_cast() {
  typedef typename internal::unpacket_traits<FromPacket>::type FromScalar;
  typedef typename internal::packet_traits<FromScalar> FromPacketTraits;
  typedef typename internal::packet_traits<ToScalar>::type Full;
  typedef typename internal::unpacket_traits<Full>::half Half;
  typedef typename internal::unpacket_traits<typename internal::unpacket_traits<Full>::half>::half Quarter;

  static const int PacketSize = internal::unpacket_traits<FromPacket>::size;
  static const bool CanCast =
      FromPacketTraits::HasCast &&
      (PacketSize == internal::unpacket_traits<Full>::size ||
      PacketSize == internal::unpacket_traits<Half>::size ||
      PacketSize == internal::unpacket_traits<Quarter>::size);

  typedef typename internal::conditional<internal::unpacket_traits<Quarter>::size == PacketSize, Quarter,
      typename internal::conditional<internal::unpacket_traits<Half>::size == PacketSize, Half, Full>::type>::type
      ToPacket;

  test_cast_helper<FromScalar, FromPacket, ToScalar, ToPacket, CanCast>::run();
}

template<typename Scalar,typename Packet> void packetmath_boolean()
{
  const int PacketSize = internal::unpacket_traits<Packet>::size;
  const int size = 2*PacketSize;
  EIGEN_ALIGN_MAX Scalar data1[size];
  EIGEN_ALIGN_MAX Scalar data2[size];
  EIGEN_ALIGN_MAX Scalar ref[size];

  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>();
  }
  CHECK_CWISE2_IF(true, internal::por, internal::por);
  CHECK_CWISE2_IF(true, internal::pxor, internal::pxor);
  CHECK_CWISE2_IF(true, internal::pand, internal::pand);
}

template<typename Scalar,typename Packet> void packetmath()
{
  typedef internal::packet_traits<Scalar> PacketTraits;
  const int PacketSize = internal::unpacket_traits<Packet>::size;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  if (g_first_pass)
    std::cerr << "=== Testing packet of type '" << typeid(Packet).name()
              << "' and scalar type '" << typeid(Scalar).name()
              << "' and size '" << PacketSize << "' ===\n" ;

  const int max_size = PacketSize > 4 ? PacketSize : 4;
  const int size = PacketSize*max_size;
  EIGEN_ALIGN_MAX Scalar data1[size];
  EIGEN_ALIGN_MAX Scalar data2[size];
  EIGEN_ALIGN_MAX Scalar data3[size];
  EIGEN_ALIGN_MAX Packet packets[PacketSize*2];
  EIGEN_ALIGN_MAX Scalar ref[size];
  RealScalar refvalue = RealScalar(0);
  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>()/RealScalar(PacketSize);
    data2[i] = internal::random<Scalar>()/RealScalar(PacketSize);
    refvalue = (std::max)(refvalue, numext::abs(data1[i]));
  }

  internal::pstore(data2, internal::pload<Packet>(data1));
  VERIFY(test::areApprox(data1, data2, PacketSize) && "aligned load/store");

  for (int offset=0; offset<PacketSize; ++offset)
  {
    internal::pstore(data2, internal::ploadu<Packet>(data1+offset));
    VERIFY(test::areApprox(data1+offset, data2, PacketSize) && "internal::ploadu");
  }

  for (int offset=0; offset<PacketSize; ++offset)
  {
    internal::pstoreu(data2+offset, internal::pload<Packet>(data1));
    VERIFY(test::areApprox(data1, data2+offset, PacketSize) && "internal::pstoreu");
  }

  if (internal::unpacket_traits<Packet>::masked_load_available)
  {
    test::packet_helper<internal::unpacket_traits<Packet>::masked_load_available, Packet> h;
    unsigned long long max_umask = (0x1ull << PacketSize);

    for (int offset=0; offset<PacketSize; ++offset)
    {
      for (unsigned long long umask=0; umask<max_umask; ++umask)
      {
        h.store(data2, h.load(data1+offset, umask));
        for (int k=0; k<PacketSize; ++k)
          data3[k] = ((umask & ( 0x1ull << k )) >> k) ? data1[k+offset] : Scalar(0);
        VERIFY(test::areApprox(data3, data2, PacketSize) && "internal::ploadu masked");
      }
    }
  }

  if (internal::unpacket_traits<Packet>::masked_store_available)
  {
    test::packet_helper<internal::unpacket_traits<Packet>::masked_store_available, Packet> h;
    unsigned long long max_umask = (0x1ull << PacketSize);

    for (int offset=0; offset<PacketSize; ++offset)
    {
      for (unsigned long long umask=0; umask<max_umask; ++umask)
      {
        internal::pstore(data2, internal::pset1<Packet>(Scalar(0)));
        h.store(data2, h.loadu(data1+offset), umask);
        for (int k=0; k<PacketSize; ++k)
          data3[k] = ((umask & ( 0x1ull << k )) >> k) ? data1[k+offset] : Scalar(0);
        VERIFY(test::areApprox(data3, data2, PacketSize) && "internal::pstoreu masked");
      }
    }
  }

  for (int offset=0; offset<PacketSize; ++offset)
  {
    #define MIN(A,B) (A<B?A:B)
    packets[0] = internal::pload<Packet>(data1);
    packets[1] = internal::pload<Packet>(data1+PacketSize);
         if (offset==0) internal::palign<0>(packets[0], packets[1]);
    else if (offset==1) internal::palign<MIN(1,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==2) internal::palign<MIN(2,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==3) internal::palign<MIN(3,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==4) internal::palign<MIN(4,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==5) internal::palign<MIN(5,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==6) internal::palign<MIN(6,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==7) internal::palign<MIN(7,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==8) internal::palign<MIN(8,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==9) internal::palign<MIN(9,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==10) internal::palign<MIN(10,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==11) internal::palign<MIN(11,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==12) internal::palign<MIN(12,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==13) internal::palign<MIN(13,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==14) internal::palign<MIN(14,PacketSize-1)>(packets[0], packets[1]);
    else if (offset==15) internal::palign<MIN(15,PacketSize-1)>(packets[0], packets[1]);
    internal::pstore(data2, packets[0]);

    for (int i=0; i<PacketSize; ++i)
      ref[i] = data1[i+offset];

    // palign is not used anymore, so let's just put a warning if it fails
    ++g_test_level;
    VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::palign");
    --g_test_level;
  }

  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasAdd);
  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasSub);
  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasMul);

  CHECK_CWISE2_IF(PacketTraits::HasAdd, REF_ADD,  internal::padd);
  CHECK_CWISE2_IF(PacketTraits::HasSub, REF_SUB,  internal::psub);
  CHECK_CWISE2_IF(PacketTraits::HasMul, REF_MUL,  internal::pmul);
  CHECK_CWISE2_IF(PacketTraits::HasDiv, REF_DIV, internal::pdiv);

  CHECK_CWISE1(internal::pnot, internal::pnot);
  CHECK_CWISE1(internal::pzero, internal::pzero);
  CHECK_CWISE1(internal::ptrue, internal::ptrue);
  if (PacketTraits::HasNegate)
    CHECK_CWISE1(internal::negate, internal::pnegate);
  CHECK_CWISE1(numext::conj, internal::pconj);

  for(int offset=0;offset<3;++offset)
  {
    for (int i=0; i<PacketSize; ++i)
      ref[i] = data1[offset];
    internal::pstore(data2, internal::pset1<Packet>(data1[offset]));
    VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::pset1");
  }

  {
    for (int i=0; i<PacketSize*4; ++i)
      ref[i] = data1[i/PacketSize];
    Packet A0, A1, A2, A3;
    internal::pbroadcast4<Packet>(data1, A0, A1, A2, A3);
    internal::pstore(data2+0*PacketSize, A0);
    internal::pstore(data2+1*PacketSize, A1);
    internal::pstore(data2+2*PacketSize, A2);
    internal::pstore(data2+3*PacketSize, A3);
    VERIFY(test::areApprox(ref, data2, 4*PacketSize) && "internal::pbroadcast4");
  }

  {
    for (int i=0; i<PacketSize*2; ++i)
      ref[i] = data1[i/PacketSize];
    Packet A0, A1;
    internal::pbroadcast2<Packet>(data1, A0, A1);
    internal::pstore(data2+0*PacketSize, A0);
    internal::pstore(data2+1*PacketSize, A1);
    VERIFY(test::areApprox(ref, data2, 2*PacketSize) && "internal::pbroadcast2");
  }

  VERIFY(internal::isApprox(data1[0], internal::pfirst(internal::pload<Packet>(data1))) && "internal::pfirst");

  if(PacketSize>1)
  {
    // apply different offsets to check that ploaddup is robust to unaligned inputs
    for(int offset=0;offset<4;++offset)
    {
      for(int i=0;i<PacketSize/2;++i)
        ref[2*i+0] = ref[2*i+1] = data1[offset+i];
      internal::pstore(data2,internal::ploaddup<Packet>(data1+offset));
      VERIFY(test::areApprox(ref, data2, PacketSize) && "ploaddup");
    }
  }

  if(PacketSize>2)
  {
    // apply different offsets to check that ploadquad is robust to unaligned inputs
    for(int offset=0;offset<4;++offset)
    {
      for(int i=0;i<PacketSize/4;++i)
        ref[4*i+0] = ref[4*i+1] = ref[4*i+2] = ref[4*i+3] = data1[offset+i];
      internal::pstore(data2,internal::ploadquad<Packet>(data1+offset));
      VERIFY(test::areApprox(ref, data2, PacketSize) && "ploadquad");
    }
  }

  ref[0] = Scalar(0);
  for (int i=0; i<PacketSize; ++i)
    ref[0] += data1[i];
  VERIFY(test::isApproxAbs(ref[0], internal::predux(internal::pload<Packet>(data1)), refvalue) && "internal::predux");

  if(PacketSize==8 && internal::unpacket_traits<typename internal::unpacket_traits<Packet>::half>::size ==4) // so far, predux_half_downto4 is only required in such a case
  {
    int HalfPacketSize = PacketSize>4 ? PacketSize/2 : PacketSize;
    for (int i=0; i<HalfPacketSize; ++i)
      ref[i] = Scalar(0);
    for (int i=0; i<PacketSize; ++i)
      ref[i%HalfPacketSize] += data1[i];
    internal::pstore(data2, internal::predux_half_dowto4(internal::pload<Packet>(data1)));
    VERIFY(test::areApprox(ref, data2, HalfPacketSize) && "internal::predux_half_dowto4");
  }

  ref[0] = Scalar(1);
  for (int i=0; i<PacketSize; ++i)
    ref[0] *= data1[i];
  VERIFY(internal::isApprox(ref[0], internal::predux_mul(internal::pload<Packet>(data1))) && "internal::predux_mul");

  if (PacketTraits::HasReduxp)
  {
    for (int j=0; j<PacketSize; ++j)
    {
      ref[j] = Scalar(0);
      for (int i=0; i<PacketSize; ++i)
        ref[j] += data1[i+j*PacketSize];
      packets[j] = internal::pload<Packet>(data1+j*PacketSize);
    }
    internal::pstore(data2, internal::preduxp(packets));
    VERIFY(test::areApproxAbs(ref, data2, PacketSize, refvalue) && "internal::preduxp");
  }

  for (int i=0; i<PacketSize; ++i)
    ref[i] = data1[PacketSize-i-1];
  internal::pstore(data2, internal::preverse(internal::pload<Packet>(data1)));
  VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::preverse");

  internal::PacketBlock<Packet> kernel;
  for (int i=0; i<PacketSize; ++i) {
    kernel.packet[i] = internal::pload<Packet>(data1+i*PacketSize);
  }
  ptranspose(kernel);
  for (int i=0; i<PacketSize; ++i) {
    internal::pstore(data2, kernel.packet[i]);
    for (int j = 0; j < PacketSize; ++j) {
      VERIFY(test::isApproxAbs(data2[j], data1[i+j*PacketSize], refvalue) && "ptranspose");
    }
  }

  if (PacketTraits::HasBlend) {
    Packet thenPacket = internal::pload<Packet>(data1);
    Packet elsePacket = internal::pload<Packet>(data2);
    EIGEN_ALIGN_MAX internal::Selector<PacketSize> selector;
    for (int i = 0; i < PacketSize; ++i) {
      selector.select[i] = i;
    }

    Packet blend = internal::pblend(selector, thenPacket, elsePacket);
    EIGEN_ALIGN_MAX Scalar result[size];
    internal::pstore(result, blend);
    for (int i = 0; i < PacketSize; ++i) {
      VERIFY(test::isApproxAbs(result[i], (selector.select[i] ? data1[i] : data2[i]), refvalue));
    }
  }

  if (PacketTraits::HasInsert || g_vectorize_sse) {
    // pinsertfirst
    for (int i=0; i<PacketSize; ++i)
      ref[i] = data1[i];
    Scalar s = internal::random<Scalar>();
    ref[0] = s;
    internal::pstore(data2, internal::pinsertfirst(internal::pload<Packet>(data1),s));
    VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::pinsertfirst");
  }

  if (PacketTraits::HasInsert || g_vectorize_sse) {
    // pinsertlast
    for (int i=0; i<PacketSize; ++i)
      ref[i] = data1[i];
    Scalar s = internal::random<Scalar>();
    ref[PacketSize-1] = s;
    internal::pstore(data2, internal::pinsertlast(internal::pload<Packet>(data1),s));
    VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::pinsertlast");
  }

  {
    for (int i = 0; i < PacketSize; ++i) {
      // "if" mask
      unsigned char v = internal::random<bool>() ? 0xff : 0;
      char* bytes = (char*)(data1+i);
      for(int k=0; k<int(sizeof(Scalar)); ++k) {
        bytes[k] = v;
      }
      // "then" packet
      data1[i+PacketSize] = internal::random<Scalar>();
      // "else" packet
      data1[i+2*PacketSize] = internal::random<Scalar>();
    }
    CHECK_CWISE3_IF(true, internal::pselect, internal::pselect);
  }

  {
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = Scalar(i);
      data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
    }
    CHECK_CWISE2_IF(true, internal::pcmp_eq, internal::pcmp_eq);
  }

  CHECK_CWISE1_IF(PacketTraits::HasSqrt, numext::sqrt, internal::psqrt);

  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>();
  }
  CHECK_CWISE2_IF(true, internal::pandnot, internal::pandnot);

  packetmath_boolean<Scalar, Packet>();
}


template<typename Scalar,typename Packet> void packetmath_real()
{
  typedef internal::packet_traits<Scalar> PacketTraits;
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  const int size = PacketSize*4;
  EIGEN_ALIGN_MAX Scalar data1[PacketSize*4];
  EIGEN_ALIGN_MAX Scalar data2[PacketSize*4];
  EIGEN_ALIGN_MAX Scalar ref[PacketSize*4];

  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>(0,1) * std::pow(Scalar(10), internal::random<Scalar>(-6,6));
    data2[i] = internal::random<Scalar>(0,1) * std::pow(Scalar(10), internal::random<Scalar>(-6,6));
  }

  if(internal::random<float>(0,1)<0.1f)
     data1[internal::random<int>(0, PacketSize)] = 0;

  CHECK_CWISE1_IF(PacketTraits::HasLog, std::log, internal::plog);
  CHECK_CWISE1_IF(PacketTraits::HasRsqrt, Scalar(1)/std::sqrt, internal::prsqrt);

  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>(-1,1) * std::pow(Scalar(10), internal::random<Scalar>(-3,3));
    data2[i] = internal::random<Scalar>(-1,1) * std::pow(Scalar(10), internal::random<Scalar>(-3,3));
  }
  CHECK_CWISE1_IF(PacketTraits::HasSin, std::sin, internal::psin);
  CHECK_CWISE1_IF(PacketTraits::HasCos, std::cos, internal::pcos);
  CHECK_CWISE1_IF(PacketTraits::HasTan, std::tan, internal::ptan);

  CHECK_CWISE1_IF(PacketTraits::HasRound, numext::round, internal::pround);
  CHECK_CWISE1_IF(PacketTraits::HasCeil, numext::ceil, internal::pceil);
  CHECK_CWISE1_IF(PacketTraits::HasFloor, numext::floor, internal::pfloor);
  CHECK_CWISE1_IF(PacketTraits::HasRint, numext::rint, internal::print);

  // See bug 1785.
  for (int i=0; i<size; ++i)
   {
     data1[i] = -1.5 + i;
     data2[i] = -1.5 + i;
   }
  CHECK_CWISE1_IF(PacketTraits::HasRound, numext::round, internal::pround);
  CHECK_CWISE1_IF(PacketTraits::HasRint, numext::rint, internal::print);

  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>(-1,1);
    data2[i] = internal::random<Scalar>(-1,1);
  }
  CHECK_CWISE1_IF(PacketTraits::HasASin, std::asin, internal::pasin);
  CHECK_CWISE1_IF(PacketTraits::HasACos, std::acos, internal::pacos);

  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>(-87,88);
    data2[i] = internal::random<Scalar>(-87,88);
  }
  CHECK_CWISE1_IF(PacketTraits::HasExp, std::exp, internal::pexp);
  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>(-1,1) * std::pow(Scalar(10), internal::random<Scalar>(-6,6));
    data2[i] = internal::random<Scalar>(-1,1) * std::pow(Scalar(10), internal::random<Scalar>(-6,6));
  }
  data1[0] = 1e-20;
  CHECK_CWISE1_IF(PacketTraits::HasTanh, std::tanh, internal::ptanh);
  if(PacketTraits::HasExp && PacketSize>=2)
  {
    data1[0] = std::numeric_limits<Scalar>::quiet_NaN();
    data1[1] = std::numeric_limits<Scalar>::epsilon();
    test::packet_helper<PacketTraits::HasExp,Packet> h;
    h.store(data2, internal::pexp(h.load(data1)));
    VERIFY((numext::isnan)(data2[0]));
    VERIFY_IS_EQUAL(std::exp(std::numeric_limits<Scalar>::epsilon()), data2[1]);

    data1[0] = -std::numeric_limits<Scalar>::epsilon();
    data1[1] = 0;
    h.store(data2, internal::pexp(h.load(data1)));
    VERIFY_IS_EQUAL(std::exp(-std::numeric_limits<Scalar>::epsilon()), data2[0]);
    VERIFY_IS_EQUAL(std::exp(Scalar(0)), data2[1]);

    data1[0] = (std::numeric_limits<Scalar>::min)();
    data1[1] = -(std::numeric_limits<Scalar>::min)();
    h.store(data2, internal::pexp(h.load(data1)));
    VERIFY_IS_EQUAL(std::exp((std::numeric_limits<Scalar>::min)()), data2[0]);
    VERIFY_IS_EQUAL(std::exp(-(std::numeric_limits<Scalar>::min)()), data2[1]);

    data1[0] = std::numeric_limits<Scalar>::denorm_min();
    data1[1] = -std::numeric_limits<Scalar>::denorm_min();
    h.store(data2, internal::pexp(h.load(data1)));
    VERIFY_IS_EQUAL(std::exp(std::numeric_limits<Scalar>::denorm_min()), data2[0]);
    VERIFY_IS_EQUAL(std::exp(-std::numeric_limits<Scalar>::denorm_min()), data2[1]);
  }

  if (PacketTraits::HasTanh) {
    // NOTE this test migh fail with GCC prior to 6.3, see MathFunctionsImpl.h for details.
    data1[0] = std::numeric_limits<Scalar>::quiet_NaN();
    test::packet_helper<internal::packet_traits<Scalar>::HasTanh,Packet> h;
    h.store(data2, internal::ptanh(h.load(data1)));
    VERIFY((numext::isnan)(data2[0]));
  }

#if EIGEN_HAS_C99_MATH && (__cplusplus > 199711L)
  data1[0] = std::numeric_limits<Scalar>::infinity();
  data1[1] = Scalar(-1);
  CHECK_CWISE1_IF(PacketTraits::HasLog1p, std::log1p, internal::plog1p);
  data1[0] = std::numeric_limits<Scalar>::infinity();
  data1[1] = -std::numeric_limits<Scalar>::infinity();
  CHECK_CWISE1_IF(PacketTraits::HasExpm1, std::expm1, internal::pexpm1);
#endif

  if(PacketSize>=2)
  {
    data1[0] = std::numeric_limits<Scalar>::quiet_NaN();
    data1[1] = std::numeric_limits<Scalar>::epsilon();
    if(PacketTraits::HasLog)
    {
      test::packet_helper<PacketTraits::HasLog,Packet> h;
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY_IS_EQUAL(std::log(std::numeric_limits<Scalar>::epsilon()), data2[1]);

      data1[0] = -std::numeric_limits<Scalar>::epsilon();
      data1[1] = 0;
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY_IS_EQUAL(std::log(Scalar(0)), data2[1]);

      data1[0] = (std::numeric_limits<Scalar>::min)();
      data1[1] = -(std::numeric_limits<Scalar>::min)();
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY_IS_EQUAL(std::log((std::numeric_limits<Scalar>::min)()), data2[0]);
      VERIFY((numext::isnan)(data2[1]));

      data1[0] = std::numeric_limits<Scalar>::denorm_min();
      data1[1] = -std::numeric_limits<Scalar>::denorm_min();
      h.store(data2, internal::plog(h.load(data1)));
      // VERIFY_IS_EQUAL(std::log(std::numeric_limits<Scalar>::denorm_min()), data2[0]);
      VERIFY((numext::isnan)(data2[1]));

      data1[0] = Scalar(-1.0f);
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));

      data1[0] = std::numeric_limits<Scalar>::infinity();
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY((numext::isinf)(data2[0]));
    }
    if(PacketTraits::HasLog1p) {
      test::packet_helper<PacketTraits::HasLog1p,Packet> h;
      data1[0] = Scalar(-2);
      data1[1] = -std::numeric_limits<Scalar>::infinity();
      h.store(data2, internal::plog1p(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY((numext::isnan)(data2[1]));
    }
    if(PacketTraits::HasSqrt)
    {
      test::packet_helper<PacketTraits::HasSqrt,Packet> h;
      data1[0] = Scalar(-1.0f);
      data1[1] = -std::numeric_limits<Scalar>::denorm_min();
      h.store(data2, internal::psqrt(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY((numext::isnan)(data2[1]));
    }
    if(PacketTraits::HasCos)
    {
      test::packet_helper<PacketTraits::HasCos,Packet> h;
      for(Scalar k = 1; k<Scalar(10000)/std::numeric_limits<Scalar>::epsilon(); k*=2)
      {
        for(int k1=0;k1<=1; ++k1)
        {
          data1[0] = (2*k+k1  )*Scalar(EIGEN_PI)/2 * internal::random<Scalar>(0.8,1.2);
          data1[1] = (2*k+2+k1)*Scalar(EIGEN_PI)/2 * internal::random<Scalar>(0.8,1.2);
          h.store(data2,            internal::pcos(h.load(data1)));
          h.store(data2+PacketSize, internal::psin(h.load(data1)));
          VERIFY(data2[0]<=Scalar(1.) && data2[0]>=Scalar(-1.));
          VERIFY(data2[1]<=Scalar(1.) && data2[1]>=Scalar(-1.));
          VERIFY(data2[PacketSize+0]<=Scalar(1.) && data2[PacketSize+0]>=Scalar(-1.));
          VERIFY(data2[PacketSize+1]<=Scalar(1.) && data2[PacketSize+1]>=Scalar(-1.));

          VERIFY_IS_APPROX(numext::abs2(data2[0])+numext::abs2(data2[PacketSize+0]), Scalar(1));
          VERIFY_IS_APPROX(numext::abs2(data2[1])+numext::abs2(data2[PacketSize+1]), Scalar(1));
        }
      }

      data1[0] =  std::numeric_limits<Scalar>::infinity();
      data1[1] = -std::numeric_limits<Scalar>::infinity();
      h.store(data2, internal::psin(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY((numext::isnan)(data2[1]));

      h.store(data2, internal::pcos(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY((numext::isnan)(data2[1]));

      data1[0] =  std::numeric_limits<Scalar>::quiet_NaN();
      h.store(data2, internal::psin(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      h.store(data2, internal::pcos(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));

      data1[0] = -Scalar(0.);
      h.store(data2, internal::psin(h.load(data1)));
      VERIFY( internal::biteq(data2[0], data1[0]) );
      h.store(data2, internal::pcos(h.load(data1)));
      VERIFY_IS_EQUAL(data2[0], Scalar(1));
    }
  }
}

template<typename Scalar,typename Packet> void packetmath_notcomplex()
{
  typedef internal::packet_traits<Scalar> PacketTraits;
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  EIGEN_ALIGN_MAX Scalar data1[PacketSize*4];
  EIGEN_ALIGN_MAX Scalar data2[PacketSize*4];
  EIGEN_ALIGN_MAX Scalar ref[PacketSize*4];

  Array<Scalar,Dynamic,1>::Map(data1, PacketSize*4).setRandom();

  if (PacketTraits::HasCast) {
    test_cast<Packet, float>();
    test_cast<Packet, double>();
    test_cast<Packet, int8_t>();
    test_cast<Packet, uint8_t>();
    test_cast<Packet, int16_t>();
    test_cast<Packet, uint16_t>();
    test_cast<Packet, int32_t>();
    test_cast<Packet, uint32_t>();
    test_cast<Packet, int64_t>();
    test_cast<Packet, uint64_t>();
  }

  ref[0] = data1[0];
  for (int i=0; i<PacketSize; ++i)
    ref[0] = (std::min)(ref[0],data1[i]);
  VERIFY(internal::isApprox(ref[0], internal::predux_min(internal::pload<Packet>(data1))) && "internal::predux_min");

  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasMin);
  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasMax);

  CHECK_CWISE2_IF(PacketTraits::HasMin, (std::min), internal::pmin);
  CHECK_CWISE2_IF(PacketTraits::HasMax, (std::max), internal::pmax);
  CHECK_CWISE1(numext::abs, internal::pabs);
  CHECK_CWISE2_IF(PacketTraits::HasAbsDiff, REF_ABS_DIFF, internal::pabsdiff);

  ref[0] = data1[0];
  for (int i=0; i<PacketSize; ++i)
    ref[0] = (std::max)(ref[0],data1[i]);
  VERIFY(internal::isApprox(ref[0], internal::predux_max(internal::pload<Packet>(data1))) && "internal::predux_max");

  for (int i=0; i<PacketSize; ++i)
    ref[i] = data1[0]+Scalar(i);
  internal::pstore(data2, internal::plset<Packet>(data1[0]));
  VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::plset");

  {
    unsigned char* data1_bits = reinterpret_cast<unsigned char*>(data1);
    // predux_all - not needed yet
    // for (unsigned int i=0; i<PacketSize*sizeof(Scalar); ++i) data1_bits[i] = 0xff;
    // VERIFY(internal::predux_all(internal::pload<Packet>(data1)) && "internal::predux_all(1111)");
    // for(int k=0; k<PacketSize; ++k)
    // {
    //   for (unsigned int i=0; i<sizeof(Scalar); ++i) data1_bits[k*sizeof(Scalar)+i] = 0x0;
    //   VERIFY( (!internal::predux_all(internal::pload<Packet>(data1))) && "internal::predux_all(0101)");
    //   for (unsigned int i=0; i<sizeof(Scalar); ++i) data1_bits[k*sizeof(Scalar)+i] = 0xff;
    // }

    // predux_any
    for (unsigned int i=0; i<PacketSize*sizeof(Scalar); ++i) data1_bits[i] = 0x0;
    VERIFY( (!internal::predux_any(internal::pload<Packet>(data1))) && "internal::predux_any(0000)");
    for(int k=0; k<PacketSize; ++k)
    {
      for (unsigned int i=0; i<sizeof(Scalar); ++i) data1_bits[k*sizeof(Scalar)+i] = 0xff;
      VERIFY( internal::predux_any(internal::pload<Packet>(data1)) && "internal::predux_any(0101)");
      for (unsigned int i=0; i<sizeof(Scalar); ++i) data1_bits[k*sizeof(Scalar)+i] = 0x00;
    }
  }
}

template<typename Scalar,typename Packet,bool ConjLhs,bool ConjRhs> void test_conj_helper(Scalar* data1, Scalar* data2, Scalar* ref, Scalar* pval)
{
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  internal::conj_if<ConjLhs> cj0;
  internal::conj_if<ConjRhs> cj1;
  internal::conj_helper<Scalar,Scalar,ConjLhs,ConjRhs> cj;
  internal::conj_helper<Packet,Packet,ConjLhs,ConjRhs> pcj;

  for(int i=0;i<PacketSize;++i)
  {
    ref[i] = cj0(data1[i]) * cj1(data2[i]);
    VERIFY(internal::isApprox(ref[i], cj.pmul(data1[i],data2[i])) && "conj_helper pmul");
  }
  internal::pstore(pval,pcj.pmul(internal::pload<Packet>(data1),internal::pload<Packet>(data2)));
  VERIFY(test::areApprox(ref, pval, PacketSize) && "conj_helper pmul");

  for(int i=0;i<PacketSize;++i)
  {
    Scalar tmp = ref[i];
    ref[i] += cj0(data1[i]) * cj1(data2[i]);
    VERIFY(internal::isApprox(ref[i], cj.pmadd(data1[i],data2[i],tmp)) && "conj_helper pmadd");
  }
  internal::pstore(pval,pcj.pmadd(internal::pload<Packet>(data1),internal::pload<Packet>(data2),internal::pload<Packet>(pval)));
  VERIFY(test::areApprox(ref, pval, PacketSize) && "conj_helper pmadd");
}

template<typename Scalar,typename Packet> void packetmath_complex()
{
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  const int size = PacketSize*4;
  EIGEN_ALIGN_MAX Scalar data1[PacketSize*4];
  EIGEN_ALIGN_MAX Scalar data2[PacketSize*4];
  EIGEN_ALIGN_MAX Scalar ref[PacketSize*4];
  EIGEN_ALIGN_MAX Scalar pval[PacketSize*4];

  for (int i=0; i<size; ++i)
  {
    data1[i] = internal::random<Scalar>() * Scalar(1e2);
    data2[i] = internal::random<Scalar>() * Scalar(1e2);
  }

  test_conj_helper<Scalar,Packet,false,false> (data1,data2,ref,pval);
  test_conj_helper<Scalar,Packet,false,true>  (data1,data2,ref,pval);
  test_conj_helper<Scalar,Packet,true,false>  (data1,data2,ref,pval);
  test_conj_helper<Scalar,Packet,true,true>   (data1,data2,ref,pval);

  {
    for(int i=0;i<PacketSize;++i)
      ref[i] = Scalar(std::imag(data1[i]),std::real(data1[i]));
    internal::pstore(pval,internal::pcplxflip(internal::pload<Packet>(data1)));
    VERIFY(test::areApprox(ref, pval, PacketSize) && "pcplxflip");
  }
}

template<typename Scalar,typename Packet> void packetmath_scatter_gather()
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const int PacketSize = internal::unpacket_traits<Packet>::size;
  EIGEN_ALIGN_MAX Scalar data1[PacketSize];
  RealScalar refvalue = 0;
  for (int i=0; i<PacketSize; ++i) {
    data1[i] = internal::random<Scalar>()/RealScalar(PacketSize);
  }

  int stride = internal::random<int>(1,20);

  EIGEN_ALIGN_MAX Scalar buffer[PacketSize*20];
  memset(buffer, 0, 20*PacketSize*sizeof(Scalar));
  Packet packet = internal::pload<Packet>(data1);
  internal::pscatter<Scalar, Packet>(buffer, packet, stride);

  for (int i = 0; i < PacketSize*20; ++i) {
    if ((i%stride) == 0 && i<stride*PacketSize) {
      VERIFY(
          test::isApproxAbs(buffer[i], data1[i/stride], refvalue) && "pscatter");
    } else {
      VERIFY(
          test::isApproxAbs(buffer[i], Scalar(0), refvalue) && "pscatter");
    }
  }

  for (int i=0; i<PacketSize*7; ++i) {
    buffer[i] = internal::random<Scalar>()/RealScalar(PacketSize);
  }
  packet = internal::pgather<Scalar, Packet>(buffer, 7);
  internal::pstore(data1, packet);
  for (int i = 0; i < PacketSize; ++i) {
    VERIFY(test::isApproxAbs(data1[i], buffer[i*7], refvalue) && "pgather");
  }
}

namespace Eigen {
namespace test {

template<typename Scalar,typename PacketType>
struct runall<Scalar,PacketType,false,false> { // i.e. float or double
  static void run() {
    packetmath<Scalar,PacketType>();
    packetmath_scatter_gather<Scalar,PacketType>();
    packetmath_notcomplex<Scalar,PacketType>();
    packetmath_real<Scalar,PacketType>();
  }
};

template<typename Scalar,typename PacketType>
struct runall<Scalar,PacketType,false,true> { // i.e. int
  static void run() {
    packetmath<Scalar,PacketType>();
    packetmath_scatter_gather<Scalar,PacketType>();
    packetmath_notcomplex<Scalar,PacketType>();
  }
};

template<typename Scalar,typename PacketType>
struct runall<Scalar,PacketType,true,false> { // i.e. complex
  static void run() {
    packetmath<Scalar,PacketType>();
    packetmath_scatter_gather<Scalar,PacketType>();
    packetmath_complex<Scalar,PacketType>();
  }
};

}
}


EIGEN_DECLARE_TEST(packetmath)
{
  g_first_pass = true;
  for(int i = 0; i < g_repeat; i++) {

    CALL_SUBTEST_1( test::runner<float>::run() );
    CALL_SUBTEST_2( test::runner<double>::run() );
    CALL_SUBTEST_3( test::runner<int8_t>::run() );
    CALL_SUBTEST_4( test::runner<uint8_t>::run() );
    CALL_SUBTEST_5( test::runner<int16_t>::run() );
    CALL_SUBTEST_6( test::runner<uint16_t>::run() );
    CALL_SUBTEST_7( test::runner<int32_t>::run() );
    CALL_SUBTEST_8( test::runner<uint32_t>::run() );
    CALL_SUBTEST_9( test::runner<int64_t>::run() );
    CALL_SUBTEST_10( test::runner<uint64_t>::run() );
    CALL_SUBTEST_11( test::runner<std::complex<float> >::run() );
    CALL_SUBTEST_12( test::runner<std::complex<double> >::run() );
    CALL_SUBTEST_13(( packetmath<half,internal::packet_traits<half>::type>() ));
#ifdef EIGEN_PACKET_MATH_SSE_H
    CALL_SUBTEST_14(( packetmath_boolean<bool,internal::packet_traits<bool>::type>() ));
#endif
    g_first_pass = false;
  }
}
