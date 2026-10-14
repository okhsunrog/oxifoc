; ModuleID = 'spi_regression.18f8146eda39b5ed-cgu.0'
source_filename = "spi_regression.18f8146eda39b5ed-cgu.0"
target datalayout = "e-m:e-p:32:32-Fi8-i64:64-v128:64:128-a:0:32-n32-S64"
target triple = "thumbv7em-unknown-none-eabihf"

@CORE_PERIPHERALS = dso_local local_unnamed_addr constant <{}> zeroinitializer, align 1, !guid !0
@_EMBASSY_DEVICE_PERIPHERALS = dso_local local_unnamed_addr global [1 x i8] zeroinitializer, align 1, !guid !1

; Function Attrs: nounwind
define dso_local noundef range(i8 -1, 4) i8 @spi_transfer(ptr noalias nofree noundef readonly align 4 captures(none) dereferenceable(8) %device, ptr noalias nofree noundef dereferenceable(2) %read, ptr noalias nofree noundef readonly captures(address, read_provenance) dereferenceable(2) %write) unnamed_addr #0 !guid !8 {
start:
  %_8 = alloca [20 x i8], align 4
  call void @llvm.lifetime.start.p0(ptr nonnull %_8)
  store i32 2, ptr %_8, align 4
  %_9.sroa.4.0..sroa_idx = getelementptr inbounds nuw i8, ptr %_8, i32 4
  store ptr %read, ptr %_9.sroa.4.0..sroa_idx, align 4
  %_9.sroa.5.0..sroa_idx = getelementptr inbounds nuw i8, ptr %_8, i32 8
  store i32 2, ptr %_9.sroa.5.0..sroa_idx, align 4
  %_9.sroa.6.0..sroa_idx = getelementptr inbounds nuw i8, ptr %_8, i32 12
  store ptr %write, ptr %_9.sroa.6.0..sroa_idx, align 4
  %_9.sroa.7.0..sroa_idx = getelementptr inbounds nuw i8, ptr %_8, i32 16
  store i32 2, ptr %_9.sroa.7.0..sroa_idx, align 4
  %_12 = getelementptr inbounds nuw i8, ptr %device, i32 4
  tail call void @llvm.experimental.noalias.scope.decl(metadata !9)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !12)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !14)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !16)
  %_2.i.i = load ptr, ptr %_12, align 4, !alias.scope !19, !noalias !20, !nonnull !21, !noundef !21
  tail call void @llvm.experimental.noalias.scope.decl(metadata !22)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !25)
  %self.val.i.i.i.i = load i8, ptr %_2.i.i, align 1, !alias.scope !28, !noalias !29, !noundef !21
  %_0.i.i.i.i.i = and i8 %self.val.i.i.i.i, 15
  %_01.i.i.i.i.i.i = lshr i8 %self.val.i.i.i.i, 4
  %_2.i.i.i.i.i = zext nneg i8 %_01.i.i.i.i.i.i to i32
  %_6.i.i.i.i.i = shl nuw nsw i32 %_2.i.i.i.i.i, 10
  %_5.i.i.i.i.i = or disjoint i32 %_6.i.i.i.i.i, 1073872896
  %_4.i.i.i.i.i = inttoptr i32 %_5.i.i.i.i.i to ptr
  %_6.i.i.i.i = getelementptr i8, ptr %_4.i.i.i.i.i, i32 24
  %0 = or disjoint i8 %_0.i.i.i.i.i, 16
  %_15.i.i.i.i = zext nneg i8 %0 to i32
  %_19.i.i.i.i = shl nuw i32 1, %_15.i.i.i.i
  store volatile i32 %_19.i.i.i.i, ptr %_6.i.i.i.i, align 8, !noalias !30
  %_16.val.i.i.i.i = load ptr, ptr %device, align 4, !alias.scope !12, !noalias !31, !nonnull !21, !align !32
  %1 = getelementptr inbounds nuw i8, ptr %_16.val.i.i.i.i, i32 23
  %.phi.trans.insert.i.i.i69.i.i.i.i = getelementptr inbounds nuw i8, ptr %_16.val.i.i.i.i, i32 4
  br label %bb1.i.i

bb1.i.i:                                          ; preds = %bb1.i.i.backedge, %start
  %_11.sroa.0.0.idx.i = phi i32 [ 0, %start ], [ %_11.sroa.0.0.add.i, %bb1.i.i.backedge ]
  %_11.sroa.0.0.ptr.i = getelementptr inbounds nuw i8, ptr %_8, i32 %_11.sroa.0.0.idx.i
  %_7.i.i.i = icmp eq i32 %_11.sroa.0.0.idx.i, 20
  br i1 %_7.i.i.i, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit, label %bb3.i.i

bb3.i.i:                                          ; preds = %bb1.i.i
  %_11.sroa.0.0.add.i = add nuw nsw i32 %_11.sroa.0.0.idx.i, 20
  tail call void @llvm.experimental.noalias.scope.decl(metadata !33)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !36)
  %_3.i.i.i.i = load i32, ptr %_11.sroa.0.0.ptr.i, align 4, !range !39, !alias.scope !40, !noalias !41, !noundef !21
  switch i32 %_3.i.i.i.i, label %default.unreachable [
    i32 0, label %bb6.i.i.i.i
    i32 1, label %bb5.i.i.i.i
    i32 2, label %bb4.i.i.i.i
    i32 3, label %bb3.i.i.i.i
    i32 4, label %bb2.i.i.i.i
  ]

default.unreachable:                              ; preds = %bb3.i.i
  unreachable

bb6.i.i.i.i:                                      ; preds = %bb3.i.i
  %2 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 4
  %_24.0.i.i.i.i = load ptr, ptr %2, align 4, !alias.scope !40, !noalias !41, !nonnull !21, !noundef !21
  %3 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 8
  %_24.1.i.i.i.i = load i32, ptr %3, align 4, !alias.scope !40, !noalias !41, !noundef !21
  tail call void @llvm.experimental.noalias.scope.decl(metadata !45)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !48)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !51)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !53)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !56)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !58)
  %4 = load i8, ptr %1, align 1, !range !61, !alias.scope !62, !noalias !63, !noundef !21
  %_3.i.i.i.i.i.i.i.i = icmp eq i8 %4, 0
  %_10.pre.i.i.i.i.i.i.i = load ptr, ptr %.phi.trans.insert.i.i.i69.i.i.i.i, align 4, !alias.scope !64, !noalias !63
  br i1 %_3.i.i.i.i.i.i.i.i, label %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i.i.i.i.i, label %bb2.i.i.i.i.i.i.i.i

bb2.i.i.i.i.i.i.i.i:                              ; preds = %bb6.i.i.i.i
  %_32.i.i.i.i.i.i.i.i = load ptr, ptr %_10.pre.i.i.i.i.i.i.i, align 4, !noalias !65, !noundef !21
  %5 = load volatile i32, ptr %_32.i.i.i.i.i.i.i.i, align 4, !noalias !65
  %_16.i.i.i.i.i.i.i.i = and i32 %5, -65
  store volatile i32 %_16.i.i.i.i.i.i.i.i, ptr %_32.i.i.i.i.i.i.i.i, align 4, !noalias !65
  %_33.i.i.i.i.i.i.i.i = load ptr, ptr %_10.pre.i.i.i.i.i.i.i, align 4, !noalias !65, !noundef !21
  %6 = load volatile i32, ptr %_33.i.i.i.i.i.i.i.i, align 4, !noalias !65
  %_26.i.i.i.i.i.i.i.i = and i32 %6, -2049
  store volatile i32 %_26.i.i.i.i.i.i.i.i, ptr %_33.i.i.i.i.i.i.i.i, align 4, !noalias !65
  store i8 0, ptr %1, align 1, !alias.scope !62, !noalias !63
  br label %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i.i.i.i.i

_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i.i.i.i.i: ; preds = %bb2.i.i.i.i.i.i.i.i, %bb6.i.i.i.i
  %_33.i.i.i.i.i.i.i = load ptr, ptr %_10.pre.i.i.i.i.i.i.i, align 4, !noalias !66, !noundef !21
  %7 = load volatile i32, ptr %_33.i.i.i.i.i.i.i, align 4, !noalias !66
  %8 = or i32 %7, 64
  store volatile i32 %8, ptr %_33.i.i.i.i.i.i.i, align 4, !noalias !66
  %_34.i.i.i.i.i.i.i = load ptr, ptr %_10.pre.i.i.i.i.i.i.i, align 4, !noalias !66, !noundef !21
  %_26.i.i.i.i.i.i.i = getelementptr i8, ptr %_34.i.i.i.i.i.i.i, i32 8
  %_245.i.i.i.i.i.i.i = load volatile i32, ptr %_26.i.i.i.i.i.i.i, align 4, !noalias !66
  %_296.i.i.i.i.i.i.i = and i32 %_245.i.i.i.i.i.i.i, 1
  %_23.not7.i.i.i.i.i.i.i = icmp eq i32 %_296.i.i.i.i.i.i.i, 0
  br i1 %_23.not7.i.i.i.i.i.i.i, label %bb9.i.i.i.i.i.i.i, label %bb8.lr.ph.i.i.i.i.i.i.i

bb8.lr.ph.i.i.i.i.i.i.i:                          ; preds = %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i.i.i.i.i
  %_31.i.i.i.i.i.i.i = getelementptr i8, ptr %_34.i.i.i.i.i.i.i, i32 12
  br label %bb8.i.i.i.i.i.i.i

bb9.i.i.i.i.i.i.i:                                ; preds = %bb8.i.i.i.i.i.i.i, %bb8.i.i.i.i.i.i.i.1, %bb8.i.i.i.i.i.i.i.2, %bb8.i.i.i.i.i.i.i.3, %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i.i.i.i.i
  fence seq_cst
  %_5.i.i.i.i.i.i.i = load ptr, ptr %_10.pre.i.i.i.i.i.i.i, align 4, !noalias !66, !noundef !21
  %9 = getelementptr inbounds nuw i8, ptr %_24.0.i.i.i.i, i32 %_24.1.i.i.i.i
  %10 = icmp eq i32 %_24.1.i.i.i.i, 0
  br i1 %10, label %bb1.i.i.backedge, label %bb2.i2.i.i.i.i.i.i.i

bb2.i2.i.i.i.i.i.i.i:                             ; preds = %bb9.i.i.i.i.i.i.i
  %11 = getelementptr i8, ptr %_5.i.i.i.i.i.i.i, i32 8
  br label %12

12:                                               ; preds = %25, %bb2.i2.i.i.i.i.i.i.i
  %13 = load volatile i32, ptr %11, align 4
  %14 = and i32 %13, 64
  %15 = icmp eq i32 %14, 0
  br i1 %15, label %16, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

16:                                               ; preds = %12
  %17 = and i32 %13, 256
  %18 = icmp eq i32 %17, 0
  br i1 %18, label %19, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

19:                                               ; preds = %16
  %20 = and i32 %13, 32
  %21 = icmp eq i32 %20, 0
  br i1 %21, label %22, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

22:                                               ; preds = %19
  %23 = and i32 %13, 16
  %24 = icmp eq i32 %23, 0
  br i1 %24, label %25, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

25:                                               ; preds = %22
  %26 = and i32 %13, 2
  %27 = icmp eq i32 %26, 0
  br i1 %27, label %12, label %bb5.preheader.i.i.i.i.i.i.i.i

bb5.preheader.i.i.i.i.i.i.i.i:                    ; preds = %25
  %_100.i.i.i.i.i.i.i.i = getelementptr i8, ptr %_5.i.i.i.i.i.i.i, i32 12
  br label %bb9.i.i.i.i.i.i.i.i

bb9.i.i.i.i.i.i.i.i:                              ; preds = %bb28.i.i.i.i.i.i.i.i, %bb5.preheader.i.i.i.i.i.i.i.i
  %_1830.i.i.i.i.i.i.i.i = phi i1 [ true, %bb5.preheader.i.i.i.i.i.i.i.i ], [ %_18.i.i.i.i.i.i.i.i, %bb28.i.i.i.i.i.i.i.i ]
  %_1629.i.i.i.i.i.i.i.i = phi i1 [ true, %bb5.preheader.i.i.i.i.i.i.i.i ], [ %_16.i3.i.i.i.i.i.i.i, %bb28.i.i.i.i.i.i.i.i ]
  %read2.sroa.0.028.i.i.i.i.i.i.i.i = phi ptr [ %_24.0.i.i.i.i, %bb5.preheader.i.i.i.i.i.i.i.i ], [ %read2.sroa.0.1.i.i.i.i.i.i.i.i, %bb28.i.i.i.i.i.i.i.i ]
  %w.sroa.0.026.i.i.i.i.i.i.i.i = phi i32 [ 0, %bb5.preheader.i.i.i.i.i.i.i.i ], [ %w.sroa.0.1.i.i.i.i.i.i.i.i, %bb28.i.i.i.i.i.i.i.i ]
  %r.sroa.0.025.i.i.i.i.i.i.i.i = phi i32 [ 0, %bb5.preheader.i.i.i.i.i.i.i.i ], [ %r.sroa.0.1.i.i.i.i.i.i.i.i, %bb28.i.i.i.i.i.i.i.i ]
  %_22.i.i.i.i.i.i.i.i = icmp eq i32 %w.sroa.0.026.i.i.i.i.i.i.i.i, %r.sroa.0.025.i.i.i.i.i.i.i.i
  %or.cond23.i.i.i.i.i.i.i.i = select i1 %_1629.i.i.i.i.i.i.i.i, i1 %_22.i.i.i.i.i.i.i.i, i1 false
  br i1 %or.cond23.i.i.i.i.i.i.i.i, label %bb11.i.i.i.i.i.i.i.i, label %bb19.i.i.i.i.i.i.i.i

bb19.i.i.i.i.i.i.i.i:                             ; preds = %bb16.i.i.i.i.i.i.i.i, %bb47.i.i.i.i.i.i.i.i, %bb9.i.i.i.i.i.i.i.i
  %w.sroa.0.1.i.i.i.i.i.i.i.i = phi i32 [ %40, %bb16.i.i.i.i.i.i.i.i ], [ %w.sroa.0.026.i.i.i.i.i.i.i.i, %bb9.i.i.i.i.i.i.i.i ], [ %w.sroa.0.026.i.i.i.i.i.i.i.i, %bb47.i.i.i.i.i.i.i.i ]
  %_38.i.i.i.i.i.i.i.i = icmp ult i32 %r.sroa.0.025.i.i.i.i.i.i.i.i, %w.sroa.0.1.i.i.i.i.i.i.i.i
  %or.cond24.i.i.i.i.i.i.i.i = select i1 %_1830.i.i.i.i.i.i.i.i, i1 %_38.i.i.i.i.i.i.i.i, i1 false
  br i1 %or.cond24.i.i.i.i.i.i.i.i, label %bb21.i.i.i.i.i.i.i.i, label %bb28.i.i.i.i.i.i.i.i

bb11.i.i.i.i.i.i.i.i:                             ; preds = %bb9.i.i.i.i.i.i.i.i
  %28 = load volatile i32, ptr %11, align 4
  %29 = and i32 %28, 64
  %.not.i = icmp eq i32 %29, 0
  br i1 %.not.i, label %30, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

30:                                               ; preds = %bb11.i.i.i.i.i.i.i.i
  %31 = and i32 %28, 256
  %32 = icmp eq i32 %31, 0
  br i1 %32, label %33, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

33:                                               ; preds = %30
  %34 = and i32 %28, 32
  %35 = icmp eq i32 %34, 0
  br i1 %35, label %36, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

36:                                               ; preds = %33
  %37 = and i32 %28, 16
  %38 = icmp eq i32 %37, 0
  br i1 %38, label %bb47.i.i.i.i.i.i.i.i, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

bb47.i.i.i.i.i.i.i.i:                             ; preds = %36
  %39 = and i32 %28, 2
  %_82.i.i.i.i.i.i.i.i.not = icmp eq i32 %39, 0
  br i1 %_82.i.i.i.i.i.i.i.i.not, label %bb19.i.i.i.i.i.i.i.i, label %bb16.i.i.i.i.i.i.i.i

bb16.i.i.i.i.i.i.i.i:                             ; preds = %bb47.i.i.i.i.i.i.i.i
  store volatile i8 0, ptr %_100.i.i.i.i.i.i.i.i, align 4, !noalias !66
  %40 = add nuw nsw i32 %w.sroa.0.026.i.i.i.i.i.i.i.i, 1
  br label %bb19.i.i.i.i.i.i.i.i

bb28.i.i.i.i.i.i.i.i:                             ; preds = %bb25.i.i.i.i.i.i.i.i, %bb61.i.i.i.i.i.i.i.i, %bb19.i.i.i.i.i.i.i.i
  %r.sroa.0.1.i.i.i.i.i.i.i.i = phi i32 [ %54, %bb25.i.i.i.i.i.i.i.i ], [ %r.sroa.0.025.i.i.i.i.i.i.i.i, %bb19.i.i.i.i.i.i.i.i ], [ %r.sroa.0.025.i.i.i.i.i.i.i.i, %bb61.i.i.i.i.i.i.i.i ]
  %read2.sroa.0.1.i.i.i.i.i.i.i.i = phi ptr [ %read2.sroa.0.2.i.i.i.i.i.i.i.i, %bb25.i.i.i.i.i.i.i.i ], [ %read2.sroa.0.028.i.i.i.i.i.i.i.i, %bb19.i.i.i.i.i.i.i.i ], [ %read2.sroa.0.028.i.i.i.i.i.i.i.i, %bb61.i.i.i.i.i.i.i.i ]
  %_16.i3.i.i.i.i.i.i.i = icmp ult i32 %w.sroa.0.1.i.i.i.i.i.i.i.i, %_24.1.i.i.i.i
  %_18.i.i.i.i.i.i.i.i = icmp ult i32 %r.sroa.0.1.i.i.i.i.i.i.i.i, %_24.1.i.i.i.i
  %or.cond.i.i.i.i.i.i.i.i = select i1 %_16.i3.i.i.i.i.i.i.i, i1 true, i1 %_18.i.i.i.i.i.i.i.i
  br i1 %or.cond.i.i.i.i.i.i.i.i, label %bb9.i.i.i.i.i.i.i.i, label %bb1.i.i.backedge

bb21.i.i.i.i.i.i.i.i:                             ; preds = %bb19.i.i.i.i.i.i.i.i
  %41 = load volatile i32, ptr %11, align 4, !noalias !66
  %42 = and i32 %41, 64
  %43 = icmp eq i32 %42, 0
  br i1 %43, label %44, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

44:                                               ; preds = %bb21.i.i.i.i.i.i.i.i
  %45 = and i32 %41, 256
  %46 = icmp eq i32 %45, 0
  br i1 %46, label %47, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

47:                                               ; preds = %44
  %48 = and i32 %41, 32
  %49 = icmp eq i32 %48, 0
  br i1 %49, label %50, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

50:                                               ; preds = %47
  %51 = and i32 %41, 16
  %52 = icmp eq i32 %51, 0
  br i1 %52, label %bb61.i.i.i.i.i.i.i.i, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

bb61.i.i.i.i.i.i.i.i:                             ; preds = %50
  %53 = trunc i32 %41 to i1
  br i1 %53, label %bb23.i.i.i.i.i.i.i.i, label %bb28.i.i.i.i.i.i.i.i

bb23.i.i.i.i.i.i.i.i:                             ; preds = %bb61.i.i.i.i.i.i.i.i
  %_120.i.i.i.i.i.i.i.i = icmp eq ptr %read2.sroa.0.028.i.i.i.i.i.i.i.i, %9
  br i1 %_120.i.i.i.i.i.i.i.i, label %bb69.i.i.i.i.i.i.i.i, label %bb67.i.i.i.i.i.i.i.i

bb67.i.i.i.i.i.i.i.i:                             ; preds = %bb23.i.i.i.i.i.i.i.i
  %_126.i.i.i.i.i.i.i.i = getelementptr inbounds nuw i8, ptr %read2.sroa.0.028.i.i.i.i.i.i.i.i, i32 1
  %_47.i.i.i.i.i.i.i.i = load volatile i8, ptr %_100.i.i.i.i.i.i.i.i, align 4, !noalias !66
  store i8 %_47.i.i.i.i.i.i.i.i, ptr %read2.sroa.0.028.i.i.i.i.i.i.i.i, align 1, !alias.scope !67, !noalias !68
  br label %bb25.i.i.i.i.i.i.i.i

bb25.i.i.i.i.i.i.i.i:                             ; preds = %bb69.i.i.i.i.i.i.i.i, %bb67.i.i.i.i.i.i.i.i
  %read2.sroa.0.2.i.i.i.i.i.i.i.i = phi ptr [ %9, %bb69.i.i.i.i.i.i.i.i ], [ %_126.i.i.i.i.i.i.i.i, %bb67.i.i.i.i.i.i.i.i ]
  %54 = add nuw nsw i32 %r.sroa.0.025.i.i.i.i.i.i.i.i, 1
  br label %bb28.i.i.i.i.i.i.i.i

bb69.i.i.i.i.i.i.i.i:                             ; preds = %bb23.i.i.i.i.i.i.i.i
  %_49.i.i.i.i.i.i.i.i = load volatile i8, ptr %_100.i.i.i.i.i.i.i.i, align 4, !noalias !66
  br label %bb25.i.i.i.i.i.i.i.i

bb8.i.i.i.i.i.i.i:                                ; preds = %bb8.i.i.i.i.i.i.i.3, %bb8.lr.ph.i.i.i.i.i.i.i
  %_25.i.i.i.i.i.i.i = load volatile i32, ptr %_31.i.i.i.i.i.i.i, align 4, !noalias !66
  %_24.i.i.i.i.i.i.i = load volatile i32, ptr %_26.i.i.i.i.i.i.i, align 4, !noalias !66
  %_29.i.i.i.i.i.i.i = and i32 %_24.i.i.i.i.i.i.i, 1
  %_23.not.i.i.i.i.i.i.i = icmp eq i32 %_29.i.i.i.i.i.i.i, 0
  br i1 %_23.not.i.i.i.i.i.i.i, label %bb9.i.i.i.i.i.i.i, label %bb8.i.i.i.i.i.i.i.1

bb8.i.i.i.i.i.i.i.1:                              ; preds = %bb8.i.i.i.i.i.i.i
  %_25.i.i.i.i.i.i.i.1 = load volatile i32, ptr %_31.i.i.i.i.i.i.i, align 4, !noalias !66
  %_24.i.i.i.i.i.i.i.1 = load volatile i32, ptr %_26.i.i.i.i.i.i.i, align 4, !noalias !66
  %_29.i.i.i.i.i.i.i.1 = and i32 %_24.i.i.i.i.i.i.i.1, 1
  %_23.not.i.i.i.i.i.i.i.1 = icmp eq i32 %_29.i.i.i.i.i.i.i.1, 0
  br i1 %_23.not.i.i.i.i.i.i.i.1, label %bb9.i.i.i.i.i.i.i, label %bb8.i.i.i.i.i.i.i.2

bb8.i.i.i.i.i.i.i.2:                              ; preds = %bb8.i.i.i.i.i.i.i.1
  %_25.i.i.i.i.i.i.i.2 = load volatile i32, ptr %_31.i.i.i.i.i.i.i, align 4, !noalias !66
  %_24.i.i.i.i.i.i.i.2 = load volatile i32, ptr %_26.i.i.i.i.i.i.i, align 4, !noalias !66
  %_29.i.i.i.i.i.i.i.2 = and i32 %_24.i.i.i.i.i.i.i.2, 1
  %_23.not.i.i.i.i.i.i.i.2 = icmp eq i32 %_29.i.i.i.i.i.i.i.2, 0
  br i1 %_23.not.i.i.i.i.i.i.i.2, label %bb9.i.i.i.i.i.i.i, label %bb8.i.i.i.i.i.i.i.3

bb8.i.i.i.i.i.i.i.3:                              ; preds = %bb8.i.i.i.i.i.i.i.2
  %_25.i.i.i.i.i.i.i.3 = load volatile i32, ptr %_31.i.i.i.i.i.i.i, align 4, !noalias !66
  %_24.i.i.i.i.i.i.i.3 = load volatile i32, ptr %_26.i.i.i.i.i.i.i, align 4, !noalias !66
  %_29.i.i.i.i.i.i.i.3 = and i32 %_24.i.i.i.i.i.i.i.3, 1
  %_23.not.i.i.i.i.i.i.i.3 = icmp eq i32 %_29.i.i.i.i.i.i.i.3, 0
  br i1 %_23.not.i.i.i.i.i.i.i.3, label %bb9.i.i.i.i.i.i.i, label %bb8.i.i.i.i.i.i.i

bb5.i.i.i.i:                                      ; preds = %bb3.i.i
  %55 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 4
  %_22.0.i.i.i.i = load ptr, ptr %55, align 4, !alias.scope !40, !noalias !41, !nonnull !21, !noundef !21
  %56 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 8
  %_22.1.i.i.i.i = load i32, ptr %56, align 4, !alias.scope !40, !noalias !41, !noundef !21
  tail call void @llvm.experimental.noalias.scope.decl(metadata !69)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !72)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !75)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !77)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !80)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !82)
  %57 = load i8, ptr %1, align 1, !range !61, !alias.scope !85, !noalias !86, !noundef !21
  %_3.i.i.i.i2.i.i.i.i = icmp eq i8 %57, 0
  %_13.pre.i.i.i.i.i.i.i = load ptr, ptr %.phi.trans.insert.i.i.i69.i.i.i.i, align 4, !alias.scope !87, !noalias !86
  br i1 %_3.i.i.i.i2.i.i.i.i, label %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i9.i.i.i.i, label %bb2.i.i.i.i4.i.i.i.i

bb2.i.i.i.i4.i.i.i.i:                             ; preds = %bb5.i.i.i.i
  %_32.i.i.i.i5.i.i.i.i = load ptr, ptr %_13.pre.i.i.i.i.i.i.i, align 4, !noalias !88, !noundef !21
  %58 = load volatile i32, ptr %_32.i.i.i.i5.i.i.i.i, align 4, !noalias !88
  %_16.i.i.i.i6.i.i.i.i = and i32 %58, -65
  store volatile i32 %_16.i.i.i.i6.i.i.i.i, ptr %_32.i.i.i.i5.i.i.i.i, align 4, !noalias !88
  %_33.i.i.i.i7.i.i.i.i = load ptr, ptr %_13.pre.i.i.i.i.i.i.i, align 4, !noalias !88, !noundef !21
  %59 = load volatile i32, ptr %_33.i.i.i.i7.i.i.i.i, align 4, !noalias !88
  %_26.i.i.i.i8.i.i.i.i = and i32 %59, -2049
  store volatile i32 %_26.i.i.i.i8.i.i.i.i, ptr %_33.i.i.i.i7.i.i.i.i, align 4, !noalias !88
  store i8 0, ptr %1, align 1, !alias.scope !85, !noalias !86
  br label %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i9.i.i.i.i

_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i9.i.i.i.i: ; preds = %bb2.i.i.i.i4.i.i.i.i, %bb5.i.i.i.i
  %_63.i.i.i.i.i.i.i = load ptr, ptr %_13.pre.i.i.i.i.i.i.i, align 4, !noalias !89, !noundef !21
  %60 = load volatile i32, ptr %_63.i.i.i.i.i.i.i, align 4, !noalias !89
  %61 = or i32 %60, 64
  store volatile i32 %61, ptr %_63.i.i.i.i.i.i.i, align 4, !noalias !89
  %_64.i.i.i.i.i.i.i = load ptr, ptr %_13.pre.i.i.i.i.i.i.i, align 4, !noalias !89, !noundef !21
  %_29.i.i.i10.i.i.i.i = getelementptr i8, ptr %_64.i.i.i.i.i.i.i, i32 8
  %_279.i.i.i.i.i.i.i = load volatile i32, ptr %_29.i.i.i10.i.i.i.i, align 4, !noalias !89
  %_3210.i.i.i.i.i.i.i = and i32 %_279.i.i.i.i.i.i.i, 1
  %_26.not11.i.i.i.i.i.i.i = icmp eq i32 %_3210.i.i.i.i.i.i.i, 0
  br i1 %_26.not11.i.i.i.i.i.i.i, label %bb12.i.i.i.i.i.i.i, label %bb11.lr.ph.i.i.i.i.i.i.i

bb11.lr.ph.i.i.i.i.i.i.i:                         ; preds = %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i9.i.i.i.i
  %_34.i.i.i11.i.i.i.i = getelementptr i8, ptr %_64.i.i.i.i.i.i.i, i32 12
  br label %bb11.i.i.i.i.i.i.i

bb12.i.i.i.i.i.i.i:                               ; preds = %bb11.i.i.i.i.i.i.i, %bb11.i.i.i.i.i.i.i.1, %bb11.i.i.i.i.i.i.i.2, %bb11.i.i.i.i.i.i.i.3, %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i9.i.i.i.i
  fence seq_cst
  %_39.i.i.i.i.i.i.i = getelementptr inbounds nuw i8, ptr %_22.0.i.i.i.i, i32 %_22.1.i.i.i.i
  %_4812.i.i.i.i.i.i.i = icmp samesign eq i32 %_22.1.i.i.i.i, 0
  br i1 %_4812.i.i.i.i.i.i.i, label %bb1.i.i.backedge, label %bb24.i.i.i.i.i.i.i

bb11.i.i.i.i.i.i.i:                               ; preds = %bb11.i.i.i.i.i.i.i.3, %bb11.lr.ph.i.i.i.i.i.i.i
  %_28.i.i.i.i.i.i.i = load volatile i32, ptr %_34.i.i.i11.i.i.i.i, align 4, !noalias !89
  %_27.i.i.i.i.i.i.i = load volatile i32, ptr %_29.i.i.i10.i.i.i.i, align 4, !noalias !89
  %_32.i.i.i.i.i.i.i = and i32 %_27.i.i.i.i.i.i.i, 1
  %_26.not.i.i.i.i.i.i.i = icmp eq i32 %_32.i.i.i.i.i.i.i, 0
  br i1 %_26.not.i.i.i.i.i.i.i, label %bb12.i.i.i.i.i.i.i, label %bb11.i.i.i.i.i.i.i.1

bb11.i.i.i.i.i.i.i.1:                             ; preds = %bb11.i.i.i.i.i.i.i
  %_28.i.i.i.i.i.i.i.1 = load volatile i32, ptr %_34.i.i.i11.i.i.i.i, align 4, !noalias !89
  %_27.i.i.i.i.i.i.i.1 = load volatile i32, ptr %_29.i.i.i10.i.i.i.i, align 4, !noalias !89
  %_32.i.i.i.i.i.i.i.1 = and i32 %_27.i.i.i.i.i.i.i.1, 1
  %_26.not.i.i.i.i.i.i.i.1 = icmp eq i32 %_32.i.i.i.i.i.i.i.1, 0
  br i1 %_26.not.i.i.i.i.i.i.i.1, label %bb12.i.i.i.i.i.i.i, label %bb11.i.i.i.i.i.i.i.2

bb11.i.i.i.i.i.i.i.2:                             ; preds = %bb11.i.i.i.i.i.i.i.1
  %_28.i.i.i.i.i.i.i.2 = load volatile i32, ptr %_34.i.i.i11.i.i.i.i, align 4, !noalias !89
  %_27.i.i.i.i.i.i.i.2 = load volatile i32, ptr %_29.i.i.i10.i.i.i.i, align 4, !noalias !89
  %_32.i.i.i.i.i.i.i.2 = and i32 %_27.i.i.i.i.i.i.i.2, 1
  %_26.not.i.i.i.i.i.i.i.2 = icmp eq i32 %_32.i.i.i.i.i.i.i.2, 0
  br i1 %_26.not.i.i.i.i.i.i.i.2, label %bb12.i.i.i.i.i.i.i, label %bb11.i.i.i.i.i.i.i.3

bb11.i.i.i.i.i.i.i.3:                             ; preds = %bb11.i.i.i.i.i.i.i.2
  %_28.i.i.i.i.i.i.i.3 = load volatile i32, ptr %_34.i.i.i11.i.i.i.i, align 4, !noalias !89
  %_27.i.i.i.i.i.i.i.3 = load volatile i32, ptr %_29.i.i.i10.i.i.i.i, align 4, !noalias !89
  %_32.i.i.i.i.i.i.i.3 = and i32 %_27.i.i.i.i.i.i.i.3, 1
  %_26.not.i.i.i.i.i.i.i.3 = icmp eq i32 %_32.i.i.i.i.i.i.i.3, 0
  br i1 %_26.not.i.i.i.i.i.i.i.3, label %bb12.i.i.i.i.i.i.i, label %bb11.i.i.i.i.i.i.i

bb24.i.i.i.i.i.i.i:                               ; preds = %bb12.i.i.i.i.i.i.i, %bb29.i.i.i.i.i.i.i
  %iter1.sroa.0.013.i.i.i.i.i.i.i = phi ptr [ %_54.i.i.i.i.i.i.i, %bb29.i.i.i.i.i.i.i ], [ %_22.0.i.i.i.i, %bb12.i.i.i.i.i.i.i ]
  %_9.i.i.i.i.i.i.i = load ptr, ptr %_13.pre.i.i.i.i.i.i.i, align 4, !noalias !89, !noundef !21
  %_10.i.i.i.i.i.i.i = load i8, ptr %iter1.sroa.0.013.i.i.i.i.i.i.i, align 1, !alias.scope !90, !noalias !91, !noundef !21
  %62 = getelementptr i8, ptr %_9.i.i.i.i.i.i.i, i32 8
  br label %63

63:                                               ; preds = %76, %bb24.i.i.i.i.i.i.i
  %64 = load volatile i32, ptr %62, align 4
  %65 = and i32 %64, 64
  %66 = icmp eq i32 %65, 0
  br i1 %66, label %67, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

67:                                               ; preds = %63
  %68 = and i32 %64, 256
  %69 = icmp eq i32 %68, 0
  br i1 %69, label %70, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

70:                                               ; preds = %67
  %71 = and i32 %64, 32
  %72 = icmp eq i32 %71, 0
  br i1 %72, label %73, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

73:                                               ; preds = %70
  %74 = and i32 %64, 16
  %75 = icmp eq i32 %74, 0
  br i1 %75, label %76, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

76:                                               ; preds = %73
  %77 = and i32 %64, 2
  %78 = icmp eq i32 %77, 0
  br i1 %78, label %63, label %bb6.i.i.i.i.i.i.i.i

bb6.i.i.i.i.i.i.i.i:                              ; preds = %76
  %_15.i.i.i.i.i.i.i.i = getelementptr i8, ptr %_9.i.i.i.i.i.i.i, i32 12
  store volatile i8 %_10.i.i.i.i.i.i.i, ptr %_15.i.i.i.i.i.i.i.i, align 4, !noalias !89
  br label %79

79:                                               ; preds = %92, %bb6.i.i.i.i.i.i.i.i
  %80 = load volatile i32, ptr %62, align 4, !noalias !89
  %81 = and i32 %80, 64
  %82 = icmp eq i32 %81, 0
  br i1 %82, label %83, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

83:                                               ; preds = %79
  %84 = and i32 %80, 256
  %85 = icmp eq i32 %84, 0
  br i1 %85, label %86, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

86:                                               ; preds = %83
  %87 = and i32 %80, 32
  %88 = icmp eq i32 %87, 0
  br i1 %88, label %89, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

89:                                               ; preds = %86
  %90 = and i32 %80, 16
  %91 = icmp eq i32 %90, 0
  br i1 %91, label %92, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

92:                                               ; preds = %89
  %93 = trunc i32 %80 to i1
  br i1 %93, label %bb29.i.i.i.i.i.i.i, label %79

bb29.i.i.i.i.i.i.i:                               ; preds = %92
  %rx_word.i.i.i.i.i.i.i.i = load volatile i8, ptr %_15.i.i.i.i.i.i.i.i, align 4, !noalias !89
  %_54.i.i.i.i.i.i.i = getelementptr inbounds nuw i8, ptr %iter1.sroa.0.013.i.i.i.i.i.i.i, i32 1
  %_48.i.i.i.i.i.i.i = icmp eq ptr %_54.i.i.i.i.i.i.i, %_39.i.i.i.i.i.i.i
  br i1 %_48.i.i.i.i.i.i.i, label %bb1.i.i.backedge, label %bb24.i.i.i.i.i.i.i

bb4.i.i.i.i:                                      ; preds = %bb3.i.i
  %94 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 4
  %_19.0.i.i.i.i = load ptr, ptr %94, align 4, !alias.scope !40, !noalias !41, !nonnull !21, !noundef !21
  %95 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 8
  %_19.1.i.i.i.i = load i32, ptr %95, align 4, !alias.scope !40, !noalias !41, !noundef !21
  %96 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 12
  %_20.0.i.i.i.i = load ptr, ptr %96, align 4, !alias.scope !40, !noalias !41, !nonnull !21, !noundef !21
  %97 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 16
  %_20.1.i.i.i.i = load i32, ptr %97, align 4, !alias.scope !40, !noalias !41, !noundef !21
  tail call void @llvm.experimental.noalias.scope.decl(metadata !92)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !95)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !97)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !100)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !102)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !104)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !107)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !109)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !111)
  %98 = load i8, ptr %1, align 1, !range !61, !alias.scope !114, !noalias !115, !noundef !21
  %_3.i.i.i.i13.i.i.i.i = icmp eq i8 %98, 0
  %_9.pre.i.i.i.i.i.i.i = load ptr, ptr %.phi.trans.insert.i.i.i69.i.i.i.i, align 4, !alias.scope !116, !noalias !115
  br i1 %_3.i.i.i.i13.i.i.i.i, label %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i20.i.i.i.i, label %bb2.i.i.i.i15.i.i.i.i

bb2.i.i.i.i15.i.i.i.i:                            ; preds = %bb4.i.i.i.i
  %_32.i.i.i.i16.i.i.i.i = load ptr, ptr %_9.pre.i.i.i.i.i.i.i, align 4, !noalias !117, !noundef !21
  %99 = load volatile i32, ptr %_32.i.i.i.i16.i.i.i.i, align 4, !noalias !117
  %_16.i.i.i.i17.i.i.i.i = and i32 %99, -65
  store volatile i32 %_16.i.i.i.i17.i.i.i.i, ptr %_32.i.i.i.i16.i.i.i.i, align 4, !noalias !117
  %_33.i.i.i.i18.i.i.i.i = load ptr, ptr %_9.pre.i.i.i.i.i.i.i, align 4, !noalias !117, !noundef !21
  %100 = load volatile i32, ptr %_33.i.i.i.i18.i.i.i.i, align 4, !noalias !117
  %_26.i.i.i.i19.i.i.i.i = and i32 %100, -2049
  store volatile i32 %_26.i.i.i.i19.i.i.i.i, ptr %_33.i.i.i.i18.i.i.i.i, align 4, !noalias !117
  store i8 0, ptr %1, align 1, !alias.scope !114, !noalias !115
  br label %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i20.i.i.i.i

_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i20.i.i.i.i: ; preds = %bb2.i.i.i.i15.i.i.i.i, %bb4.i.i.i.i
  %_32.i.i.i21.i.i.i.i = load ptr, ptr %_9.pre.i.i.i.i.i.i.i, align 4, !noalias !118, !noundef !21
  %101 = load volatile i32, ptr %_32.i.i.i21.i.i.i.i, align 4, !noalias !118
  %102 = or i32 %101, 64
  store volatile i32 %102, ptr %_32.i.i.i21.i.i.i.i, align 4, !noalias !118
  %_33.i.i.i22.i.i.i.i = load ptr, ptr %_9.pre.i.i.i.i.i.i.i, align 4, !noalias !118, !noundef !21
  %_25.i.i.i23.i.i.i.i = getelementptr i8, ptr %_33.i.i.i22.i.i.i.i, i32 8
  %_235.i.i.i.i.i.i.i = load volatile i32, ptr %_25.i.i.i23.i.i.i.i, align 4, !noalias !118
  %_286.i.i.i.i.i.i.i = and i32 %_235.i.i.i.i.i.i.i, 1
  %_22.not7.i.i.i.i.i.i.i = icmp eq i32 %_286.i.i.i.i.i.i.i, 0
  br i1 %_22.not7.i.i.i.i.i.i.i, label %bb9.i.i.i28.i.i.i.i, label %bb8.lr.ph.i.i.i24.i.i.i.i

bb8.lr.ph.i.i.i24.i.i.i.i:                        ; preds = %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i20.i.i.i.i
  %_30.i.i.i.i.i.i.i = getelementptr i8, ptr %_33.i.i.i22.i.i.i.i, i32 12
  br label %bb8.i.i.i25.i.i.i.i

bb9.i.i.i28.i.i.i.i:                              ; preds = %bb8.i.i.i25.i.i.i.i, %bb8.i.i.i25.i.i.i.i.1, %bb8.i.i.i25.i.i.i.i.2, %bb8.i.i.i25.i.i.i.i.3, %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i20.i.i.i.i
  fence seq_cst
  %_6.i.i.i.i.i.i.i = load ptr, ptr %_9.pre.i.i.i.i.i.i.i, align 4, !noalias !118, !noundef !21
  %ndt.i.i.i.i.i.i.i.i = tail call i32 @llvm.umax.i32(i32 range(i32 0, -2147483648) %_19.1.i.i.i.i, i32 range(i32 0, -2147483648) %_20.1.i.i.i.i)
  %103 = getelementptr inbounds nuw i8, ptr %_19.0.i.i.i.i, i32 %_19.1.i.i.i.i
  %_70.i.i.i.i.i.i.i.i = getelementptr inbounds nuw i8, ptr %_20.0.i.i.i.i, i32 %_20.1.i.i.i.i
  %104 = icmp eq i32 %ndt.i.i.i.i.i.i.i.i, 0
  br i1 %104, label %bb1.i.i.backedge, label %bb2.i2.i.i.i29.i.i.i.i

bb2.i2.i.i.i29.i.i.i.i:                           ; preds = %bb9.i.i.i28.i.i.i.i
  %105 = getelementptr i8, ptr %_6.i.i.i.i.i.i.i, i32 8
  br label %106

106:                                              ; preds = %119, %bb2.i2.i.i.i29.i.i.i.i
  %107 = load volatile i32, ptr %105, align 4
  %108 = and i32 %107, 64
  %109 = icmp eq i32 %108, 0
  br i1 %109, label %110, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

110:                                              ; preds = %106
  %111 = and i32 %107, 256
  %112 = icmp eq i32 %111, 0
  br i1 %112, label %113, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

113:                                              ; preds = %110
  %114 = and i32 %107, 32
  %115 = icmp eq i32 %114, 0
  br i1 %115, label %116, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

116:                                              ; preds = %113
  %117 = and i32 %107, 16
  %118 = icmp eq i32 %117, 0
  br i1 %118, label %119, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

119:                                              ; preds = %116
  %120 = and i32 %107, 2
  %121 = icmp eq i32 %120, 0
  br i1 %121, label %106, label %bb5.preheader.i.i.i.i32.i.i.i.i

bb5.preheader.i.i.i.i32.i.i.i.i:                  ; preds = %119
  %_100.i.i.i.i33.i.i.i.i = getelementptr i8, ptr %_6.i.i.i.i.i.i.i, i32 12
  br label %bb9.i.i.i.i34.i.i.i.i

bb9.i.i.i.i34.i.i.i.i:                            ; preds = %bb28.i.i.i.i46.i.i.i.i, %bb5.preheader.i.i.i.i32.i.i.i.i
  %_1830.i.i.i.i35.i.i.i.i = phi i1 [ true, %bb5.preheader.i.i.i.i32.i.i.i.i ], [ %_18.i.i.i.i50.i.i.i.i, %bb28.i.i.i.i46.i.i.i.i ]
  %_1629.i.i.i.i36.i.i.i.i = phi i1 [ true, %bb5.preheader.i.i.i.i32.i.i.i.i ], [ %_16.i3.i.i.i49.i.i.i.i, %bb28.i.i.i.i46.i.i.i.i ]
  %read2.sroa.0.028.i.i.i.i37.i.i.i.i = phi ptr [ %_19.0.i.i.i.i, %bb5.preheader.i.i.i.i32.i.i.i.i ], [ %read2.sroa.0.1.i.i.i.i48.i.i.i.i, %bb28.i.i.i.i46.i.i.i.i ]
  %write1.sroa.0.027.i.i.i.i.i.i.i.i = phi ptr [ %_20.0.i.i.i.i, %bb5.preheader.i.i.i.i32.i.i.i.i ], [ %write1.sroa.0.1.i.i.i.i.i.i.i.i, %bb28.i.i.i.i46.i.i.i.i ]
  %w.sroa.0.026.i.i.i.i38.i.i.i.i = phi i32 [ 0, %bb5.preheader.i.i.i.i32.i.i.i.i ], [ %w.sroa.0.1.i.i.i.i43.i.i.i.i, %bb28.i.i.i.i46.i.i.i.i ]
  %r.sroa.0.025.i.i.i.i39.i.i.i.i = phi i32 [ 0, %bb5.preheader.i.i.i.i32.i.i.i.i ], [ %r.sroa.0.1.i.i.i.i47.i.i.i.i, %bb28.i.i.i.i46.i.i.i.i ]
  %_22.i.i.i.i40.i.i.i.i = icmp eq i32 %w.sroa.0.026.i.i.i.i38.i.i.i.i, %r.sroa.0.025.i.i.i.i39.i.i.i.i
  %or.cond23.i.i.i.i41.i.i.i.i = select i1 %_1629.i.i.i.i36.i.i.i.i, i1 %_22.i.i.i.i40.i.i.i.i, i1 false
  br i1 %or.cond23.i.i.i.i41.i.i.i.i, label %bb11.i.i.i.i64.i.i.i.i, label %bb19.i.i.i.i42.i.i.i.i

bb19.i.i.i.i42.i.i.i.i:                           ; preds = %bb16.i.i.i.i67.i.i.i.i, %bb47.i.i.i.i65.i.i.i.i, %bb9.i.i.i.i34.i.i.i.i
  %w.sroa.0.1.i.i.i.i43.i.i.i.i = phi i32 [ %134, %bb16.i.i.i.i67.i.i.i.i ], [ %w.sroa.0.026.i.i.i.i38.i.i.i.i, %bb9.i.i.i.i34.i.i.i.i ], [ %w.sroa.0.026.i.i.i.i38.i.i.i.i, %bb47.i.i.i.i65.i.i.i.i ]
  %write1.sroa.0.1.i.i.i.i.i.i.i.i = phi ptr [ %write1.sroa.0.2.i.i.i.i.i.i.i.i, %bb16.i.i.i.i67.i.i.i.i ], [ %write1.sroa.0.027.i.i.i.i.i.i.i.i, %bb9.i.i.i.i34.i.i.i.i ], [ %write1.sroa.0.027.i.i.i.i.i.i.i.i, %bb47.i.i.i.i65.i.i.i.i ]
  %_38.i.i.i.i44.i.i.i.i = icmp ult i32 %r.sroa.0.025.i.i.i.i39.i.i.i.i, %w.sroa.0.1.i.i.i.i43.i.i.i.i
  %or.cond24.i.i.i.i45.i.i.i.i = select i1 %_1830.i.i.i.i35.i.i.i.i, i1 %_38.i.i.i.i44.i.i.i.i, i1 false
  br i1 %or.cond24.i.i.i.i45.i.i.i.i, label %bb21.i.i.i.i52.i.i.i.i, label %bb28.i.i.i.i46.i.i.i.i

bb11.i.i.i.i64.i.i.i.i:                           ; preds = %bb9.i.i.i.i34.i.i.i.i
  %122 = load volatile i32, ptr %105, align 4
  %123 = and i32 %122, 64
  %.not.i39 = icmp eq i32 %123, 0
  br i1 %.not.i39, label %124, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

124:                                              ; preds = %bb11.i.i.i.i64.i.i.i.i
  %125 = and i32 %122, 256
  %126 = icmp eq i32 %125, 0
  br i1 %126, label %127, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

127:                                              ; preds = %124
  %128 = and i32 %122, 32
  %129 = icmp eq i32 %128, 0
  br i1 %129, label %130, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

130:                                              ; preds = %127
  %131 = and i32 %122, 16
  %132 = icmp eq i32 %131, 0
  br i1 %132, label %bb47.i.i.i.i65.i.i.i.i, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

bb47.i.i.i.i65.i.i.i.i:                           ; preds = %130
  %133 = and i32 %122, 2
  %_82.i.i.i.i66.i.i.i.i.not = icmp eq i32 %133, 0
  br i1 %_82.i.i.i.i66.i.i.i.i.not, label %bb19.i.i.i.i42.i.i.i.i, label %bb13.i.i.i.i.i.i.i.i

bb13.i.i.i.i.i.i.i.i:                             ; preds = %bb47.i.i.i.i65.i.i.i.i
  %_92.i.i.i.i.i.i.i.i = icmp eq ptr %write1.sroa.0.027.i.i.i.i.i.i.i.i, %_70.i.i.i.i.i.i.i.i
  br i1 %_92.i.i.i.i.i.i.i.i, label %bb16.i.i.i.i67.i.i.i.i, label %bb52.i.i.i.i.i.i.i.i

bb52.i.i.i.i.i.i.i.i:                             ; preds = %bb13.i.i.i.i.i.i.i.i
  %_98.i.i.i.i.i.i.i.i = getelementptr inbounds nuw i8, ptr %write1.sroa.0.027.i.i.i.i.i.i.i.i, i32 1
  %_33.i4.i.i.i.i.i.i.i = load i8, ptr %write1.sroa.0.027.i.i.i.i.i.i.i.i, align 1, !alias.scope !119, !noalias !120, !noundef !21
  br label %bb16.i.i.i.i67.i.i.i.i

bb16.i.i.i.i67.i.i.i.i:                           ; preds = %bb52.i.i.i.i.i.i.i.i, %bb13.i.i.i.i.i.i.i.i
  %.sink.i.i.i.i.i.i.i.i = phi i8 [ %_33.i4.i.i.i.i.i.i.i, %bb52.i.i.i.i.i.i.i.i ], [ 0, %bb13.i.i.i.i.i.i.i.i ]
  %write1.sroa.0.2.i.i.i.i.i.i.i.i = phi ptr [ %_98.i.i.i.i.i.i.i.i, %bb52.i.i.i.i.i.i.i.i ], [ %_70.i.i.i.i.i.i.i.i, %bb13.i.i.i.i.i.i.i.i ]
  store volatile i8 %.sink.i.i.i.i.i.i.i.i, ptr %_100.i.i.i.i33.i.i.i.i, align 4, !noalias !118
  %134 = add nuw nsw i32 %w.sroa.0.026.i.i.i.i38.i.i.i.i, 1
  br label %bb19.i.i.i.i42.i.i.i.i

bb28.i.i.i.i46.i.i.i.i:                           ; preds = %bb25.i.i.i.i60.i.i.i.i, %bb61.i.i.i.i53.i.i.i.i, %bb19.i.i.i.i42.i.i.i.i
  %r.sroa.0.1.i.i.i.i47.i.i.i.i = phi i32 [ %148, %bb25.i.i.i.i60.i.i.i.i ], [ %r.sroa.0.025.i.i.i.i39.i.i.i.i, %bb19.i.i.i.i42.i.i.i.i ], [ %r.sroa.0.025.i.i.i.i39.i.i.i.i, %bb61.i.i.i.i53.i.i.i.i ]
  %read2.sroa.0.1.i.i.i.i48.i.i.i.i = phi ptr [ %read2.sroa.0.2.i.i.i.i61.i.i.i.i, %bb25.i.i.i.i60.i.i.i.i ], [ %read2.sroa.0.028.i.i.i.i37.i.i.i.i, %bb19.i.i.i.i42.i.i.i.i ], [ %read2.sroa.0.028.i.i.i.i37.i.i.i.i, %bb61.i.i.i.i53.i.i.i.i ]
  %_16.i3.i.i.i49.i.i.i.i = icmp ult i32 %w.sroa.0.1.i.i.i.i43.i.i.i.i, %ndt.i.i.i.i.i.i.i.i
  %_18.i.i.i.i50.i.i.i.i = icmp ult i32 %r.sroa.0.1.i.i.i.i47.i.i.i.i, %ndt.i.i.i.i.i.i.i.i
  %or.cond.i.i.i.i51.i.i.i.i = select i1 %_16.i3.i.i.i49.i.i.i.i, i1 true, i1 %_18.i.i.i.i50.i.i.i.i
  br i1 %or.cond.i.i.i.i51.i.i.i.i, label %bb9.i.i.i.i34.i.i.i.i, label %bb1.i.i.backedge

bb21.i.i.i.i52.i.i.i.i:                           ; preds = %bb19.i.i.i.i42.i.i.i.i
  %135 = load volatile i32, ptr %105, align 4, !noalias !118
  %136 = and i32 %135, 64
  %137 = icmp eq i32 %136, 0
  br i1 %137, label %138, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

138:                                              ; preds = %bb21.i.i.i.i52.i.i.i.i
  %139 = and i32 %135, 256
  %140 = icmp eq i32 %139, 0
  br i1 %140, label %141, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

141:                                              ; preds = %138
  %142 = and i32 %135, 32
  %143 = icmp eq i32 %142, 0
  br i1 %143, label %144, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

144:                                              ; preds = %141
  %145 = and i32 %135, 16
  %146 = icmp eq i32 %145, 0
  br i1 %146, label %bb61.i.i.i.i53.i.i.i.i, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

bb61.i.i.i.i53.i.i.i.i:                           ; preds = %144
  %147 = trunc i32 %135 to i1
  br i1 %147, label %bb23.i.i.i.i55.i.i.i.i, label %bb28.i.i.i.i46.i.i.i.i

bb23.i.i.i.i55.i.i.i.i:                           ; preds = %bb61.i.i.i.i53.i.i.i.i
  %_120.i.i.i.i56.i.i.i.i = icmp eq ptr %read2.sroa.0.028.i.i.i.i37.i.i.i.i, %103
  br i1 %_120.i.i.i.i56.i.i.i.i, label %bb69.i.i.i.i62.i.i.i.i, label %bb67.i.i.i.i57.i.i.i.i

bb67.i.i.i.i57.i.i.i.i:                           ; preds = %bb23.i.i.i.i55.i.i.i.i
  %_126.i.i.i.i58.i.i.i.i = getelementptr inbounds nuw i8, ptr %read2.sroa.0.028.i.i.i.i37.i.i.i.i, i32 1
  %_47.i.i.i.i59.i.i.i.i = load volatile i8, ptr %_100.i.i.i.i33.i.i.i.i, align 4, !noalias !118
  store i8 %_47.i.i.i.i59.i.i.i.i, ptr %read2.sroa.0.028.i.i.i.i37.i.i.i.i, align 1, !alias.scope !121, !noalias !122
  br label %bb25.i.i.i.i60.i.i.i.i

bb25.i.i.i.i60.i.i.i.i:                           ; preds = %bb69.i.i.i.i62.i.i.i.i, %bb67.i.i.i.i57.i.i.i.i
  %read2.sroa.0.2.i.i.i.i61.i.i.i.i = phi ptr [ %103, %bb69.i.i.i.i62.i.i.i.i ], [ %_126.i.i.i.i58.i.i.i.i, %bb67.i.i.i.i57.i.i.i.i ]
  %148 = add nuw nsw i32 %r.sroa.0.025.i.i.i.i39.i.i.i.i, 1
  br label %bb28.i.i.i.i46.i.i.i.i

bb69.i.i.i.i62.i.i.i.i:                           ; preds = %bb23.i.i.i.i55.i.i.i.i
  %_49.i.i.i.i63.i.i.i.i = load volatile i8, ptr %_100.i.i.i.i33.i.i.i.i, align 4, !noalias !118
  br label %bb25.i.i.i.i60.i.i.i.i

bb8.i.i.i25.i.i.i.i:                              ; preds = %bb8.i.i.i25.i.i.i.i.3, %bb8.lr.ph.i.i.i24.i.i.i.i
  %_24.i.i.i26.i.i.i.i = load volatile i32, ptr %_30.i.i.i.i.i.i.i, align 4, !noalias !118
  %_23.i.i.i.i.i.i.i = load volatile i32, ptr %_25.i.i.i23.i.i.i.i, align 4, !noalias !118
  %_28.i.i.i27.i.i.i.i = and i32 %_23.i.i.i.i.i.i.i, 1
  %_22.not.i.i.i.i.i.i.i = icmp eq i32 %_28.i.i.i27.i.i.i.i, 0
  br i1 %_22.not.i.i.i.i.i.i.i, label %bb9.i.i.i28.i.i.i.i, label %bb8.i.i.i25.i.i.i.i.1

bb8.i.i.i25.i.i.i.i.1:                            ; preds = %bb8.i.i.i25.i.i.i.i
  %_24.i.i.i26.i.i.i.i.1 = load volatile i32, ptr %_30.i.i.i.i.i.i.i, align 4, !noalias !118
  %_23.i.i.i.i.i.i.i.1 = load volatile i32, ptr %_25.i.i.i23.i.i.i.i, align 4, !noalias !118
  %_28.i.i.i27.i.i.i.i.1 = and i32 %_23.i.i.i.i.i.i.i.1, 1
  %_22.not.i.i.i.i.i.i.i.1 = icmp eq i32 %_28.i.i.i27.i.i.i.i.1, 0
  br i1 %_22.not.i.i.i.i.i.i.i.1, label %bb9.i.i.i28.i.i.i.i, label %bb8.i.i.i25.i.i.i.i.2

bb8.i.i.i25.i.i.i.i.2:                            ; preds = %bb8.i.i.i25.i.i.i.i.1
  %_24.i.i.i26.i.i.i.i.2 = load volatile i32, ptr %_30.i.i.i.i.i.i.i, align 4, !noalias !118
  %_23.i.i.i.i.i.i.i.2 = load volatile i32, ptr %_25.i.i.i23.i.i.i.i, align 4, !noalias !118
  %_28.i.i.i27.i.i.i.i.2 = and i32 %_23.i.i.i.i.i.i.i.2, 1
  %_22.not.i.i.i.i.i.i.i.2 = icmp eq i32 %_28.i.i.i27.i.i.i.i.2, 0
  br i1 %_22.not.i.i.i.i.i.i.i.2, label %bb9.i.i.i28.i.i.i.i, label %bb8.i.i.i25.i.i.i.i.3

bb8.i.i.i25.i.i.i.i.3:                            ; preds = %bb8.i.i.i25.i.i.i.i.2
  %_24.i.i.i26.i.i.i.i.3 = load volatile i32, ptr %_30.i.i.i.i.i.i.i, align 4, !noalias !118
  %_23.i.i.i.i.i.i.i.3 = load volatile i32, ptr %_25.i.i.i23.i.i.i.i, align 4, !noalias !118
  %_28.i.i.i27.i.i.i.i.3 = and i32 %_23.i.i.i.i.i.i.i.3, 1
  %_22.not.i.i.i.i.i.i.i.3 = icmp eq i32 %_28.i.i.i27.i.i.i.i.3, 0
  br i1 %_22.not.i.i.i.i.i.i.i.3, label %bb9.i.i.i28.i.i.i.i, label %bb8.i.i.i25.i.i.i.i

bb3.i.i.i.i:                                      ; preds = %bb3.i.i
  %149 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 4
  %_17.0.i.i.i.i = load ptr, ptr %149, align 4, !alias.scope !40, !noalias !41, !nonnull !21, !noundef !21
  %150 = getelementptr inbounds nuw i8, ptr %_11.sroa.0.0.ptr.i, i32 8
  %_17.1.i.i.i.i = load i32, ptr %150, align 4, !alias.scope !40, !noalias !41, !noundef !21
  tail call void @llvm.experimental.noalias.scope.decl(metadata !123)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !126)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !129)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !131)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !134)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !136)
  %151 = load i8, ptr %1, align 1, !range !61, !alias.scope !139, !noalias !140, !noundef !21
  %_3.i.i.i.i68.i.i.i.i = icmp eq i8 %151, 0
  %_8.pre.i.i.i.i.i.i.i = load ptr, ptr %.phi.trans.insert.i.i.i69.i.i.i.i, align 4, !alias.scope !141, !noalias !140
  br i1 %_3.i.i.i.i68.i.i.i.i, label %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i75.i.i.i.i, label %bb2.i.i.i.i70.i.i.i.i

bb2.i.i.i.i70.i.i.i.i:                            ; preds = %bb3.i.i.i.i
  %_32.i.i.i.i71.i.i.i.i = load ptr, ptr %_8.pre.i.i.i.i.i.i.i, align 4, !noalias !142, !noundef !21
  %152 = load volatile i32, ptr %_32.i.i.i.i71.i.i.i.i, align 4, !noalias !142
  %_16.i.i.i.i72.i.i.i.i = and i32 %152, -65
  store volatile i32 %_16.i.i.i.i72.i.i.i.i, ptr %_32.i.i.i.i71.i.i.i.i, align 4, !noalias !142
  %_33.i.i.i.i73.i.i.i.i = load ptr, ptr %_8.pre.i.i.i.i.i.i.i, align 4, !noalias !142, !noundef !21
  %153 = load volatile i32, ptr %_33.i.i.i.i73.i.i.i.i, align 4, !noalias !142
  %_26.i.i.i.i74.i.i.i.i = and i32 %153, -2049
  store volatile i32 %_26.i.i.i.i74.i.i.i.i, ptr %_33.i.i.i.i73.i.i.i.i, align 4, !noalias !142
  store i8 0, ptr %1, align 1, !alias.scope !139, !noalias !140
  br label %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i75.i.i.i.i

_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i75.i.i.i.i: ; preds = %bb2.i.i.i.i70.i.i.i.i, %bb3.i.i.i.i
  %_31.i.i.i76.i.i.i.i = load ptr, ptr %_8.pre.i.i.i.i.i.i.i, align 4, !noalias !143, !noundef !21
  %154 = load volatile i32, ptr %_31.i.i.i76.i.i.i.i, align 4, !noalias !143
  %155 = or i32 %154, 64
  store volatile i32 %155, ptr %_31.i.i.i76.i.i.i.i, align 4, !noalias !143
  %_32.i.i.i77.i.i.i.i = load ptr, ptr %_8.pre.i.i.i.i.i.i.i, align 4, !noalias !143, !noundef !21
  %_24.i.i.i78.i.i.i.i = getelementptr i8, ptr %_32.i.i.i77.i.i.i.i, i32 8
  %_225.i.i.i.i.i.i.i = load volatile i32, ptr %_24.i.i.i78.i.i.i.i, align 4, !noalias !143
  %_276.i.i.i.i.i.i.i = and i32 %_225.i.i.i.i.i.i.i, 1
  %_21.not7.i.i.i.i.i.i.i = icmp eq i32 %_276.i.i.i.i.i.i.i, 0
  br i1 %_21.not7.i.i.i.i.i.i.i, label %bb9.i.i.i84.i.i.i.i, label %bb8.lr.ph.i.i.i79.i.i.i.i

bb8.lr.ph.i.i.i79.i.i.i.i:                        ; preds = %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i75.i.i.i.i
  %_29.i.i.i80.i.i.i.i = getelementptr i8, ptr %_32.i.i.i77.i.i.i.i, i32 12
  br label %bb8.i.i.i81.i.i.i.i

bb9.i.i.i84.i.i.i.i:                              ; preds = %bb8.i.i.i81.i.i.i.i, %bb8.i.i.i81.i.i.i.i.1, %bb8.i.i.i81.i.i.i.i.2, %bb8.i.i.i81.i.i.i.i.3, %_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression.exit.i.i.i75.i.i.i.i
  fence seq_cst
  %_5.i.i.i85.i.i.i.i = load ptr, ptr %_8.pre.i.i.i.i.i.i.i, align 4, !noalias !143, !noundef !21
  %156 = getelementptr inbounds nuw i8, ptr %_17.0.i.i.i.i, i32 %_17.1.i.i.i.i
  %157 = icmp eq i32 %_17.1.i.i.i.i, 0
  br i1 %157, label %bb1.i.i.backedge, label %bb2.i2.i.i.i86.i.i.i.i

bb1.i.i.backedge:                                 ; preds = %bb28.i.i.i.i105.i.i.i.i, %bb28.i.i.i.i46.i.i.i.i, %bb29.i.i.i.i.i.i.i, %bb28.i.i.i.i.i.i.i.i, %bb9.i.i.i84.i.i.i.i, %bb9.i.i.i28.i.i.i.i, %bb12.i.i.i.i.i.i.i, %bb9.i.i.i.i.i.i.i
  br label %bb1.i.i

bb2.i2.i.i.i86.i.i.i.i:                           ; preds = %bb9.i.i.i84.i.i.i.i
  %158 = getelementptr i8, ptr %_5.i.i.i85.i.i.i.i, i32 8
  br label %159

159:                                              ; preds = %172, %bb2.i2.i.i.i86.i.i.i.i
  %160 = load volatile i32, ptr %158, align 4
  %161 = and i32 %160, 64
  %162 = icmp eq i32 %161, 0
  br i1 %162, label %163, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

163:                                              ; preds = %159
  %164 = and i32 %160, 256
  %165 = icmp eq i32 %164, 0
  br i1 %165, label %166, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

166:                                              ; preds = %163
  %167 = and i32 %160, 32
  %168 = icmp eq i32 %167, 0
  br i1 %168, label %169, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

169:                                              ; preds = %166
  %170 = and i32 %160, 16
  %171 = icmp eq i32 %170, 0
  br i1 %171, label %172, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

172:                                              ; preds = %169
  %173 = and i32 %160, 2
  %174 = icmp eq i32 %173, 0
  br i1 %174, label %159, label %bb5.preheader.i.i.i.i89.i.i.i.i

bb5.preheader.i.i.i.i89.i.i.i.i:                  ; preds = %172
  %_100.i.i.i.i90.i.i.i.i = getelementptr i8, ptr %_5.i.i.i85.i.i.i.i, i32 12
  br label %bb9.i.i.i.i91.i.i.i.i

bb9.i.i.i.i91.i.i.i.i:                            ; preds = %bb28.i.i.i.i105.i.i.i.i, %bb5.preheader.i.i.i.i89.i.i.i.i
  %_1830.i.i.i.i92.i.i.i.i = phi i1 [ true, %bb5.preheader.i.i.i.i89.i.i.i.i ], [ %_18.i.i.i.i109.i.i.i.i, %bb28.i.i.i.i105.i.i.i.i ]
  %_1629.i.i.i.i93.i.i.i.i = phi i1 [ true, %bb5.preheader.i.i.i.i89.i.i.i.i ], [ %_16.i3.i.i.i108.i.i.i.i, %bb28.i.i.i.i105.i.i.i.i ]
  %read2.sroa.0.028.i.i.i.i94.i.i.i.i = phi ptr [ %_17.0.i.i.i.i, %bb5.preheader.i.i.i.i89.i.i.i.i ], [ %read2.sroa.0.1.i.i.i.i107.i.i.i.i, %bb28.i.i.i.i105.i.i.i.i ]
  %write1.sroa.0.027.i.i.i.i95.i.i.i.i = phi ptr [ %_17.0.i.i.i.i, %bb5.preheader.i.i.i.i89.i.i.i.i ], [ %write1.sroa.0.1.i.i.i.i102.i.i.i.i, %bb28.i.i.i.i105.i.i.i.i ]
  %w.sroa.0.026.i.i.i.i96.i.i.i.i = phi i32 [ 0, %bb5.preheader.i.i.i.i89.i.i.i.i ], [ %w.sroa.0.1.i.i.i.i101.i.i.i.i, %bb28.i.i.i.i105.i.i.i.i ]
  %r.sroa.0.025.i.i.i.i97.i.i.i.i = phi i32 [ 0, %bb5.preheader.i.i.i.i89.i.i.i.i ], [ %r.sroa.0.1.i.i.i.i106.i.i.i.i, %bb28.i.i.i.i105.i.i.i.i ]
  %_22.i.i.i.i98.i.i.i.i = icmp eq i32 %w.sroa.0.026.i.i.i.i96.i.i.i.i, %r.sroa.0.025.i.i.i.i97.i.i.i.i
  %or.cond23.i.i.i.i99.i.i.i.i = select i1 %_1629.i.i.i.i93.i.i.i.i, i1 %_22.i.i.i.i98.i.i.i.i, i1 false
  br i1 %or.cond23.i.i.i.i99.i.i.i.i, label %bb11.i.i.i.i123.i.i.i.i, label %bb19.i.i.i.i100.i.i.i.i

bb19.i.i.i.i100.i.i.i.i:                          ; preds = %bb16.i.i.i.i131.i.i.i.i, %bb47.i.i.i.i124.i.i.i.i, %bb9.i.i.i.i91.i.i.i.i
  %w.sroa.0.1.i.i.i.i101.i.i.i.i = phi i32 [ %187, %bb16.i.i.i.i131.i.i.i.i ], [ %w.sroa.0.026.i.i.i.i96.i.i.i.i, %bb9.i.i.i.i91.i.i.i.i ], [ %w.sroa.0.026.i.i.i.i96.i.i.i.i, %bb47.i.i.i.i124.i.i.i.i ]
  %write1.sroa.0.1.i.i.i.i102.i.i.i.i = phi ptr [ %write1.sroa.0.2.i.i.i.i133.i.i.i.i, %bb16.i.i.i.i131.i.i.i.i ], [ %write1.sroa.0.027.i.i.i.i95.i.i.i.i, %bb9.i.i.i.i91.i.i.i.i ], [ %write1.sroa.0.027.i.i.i.i95.i.i.i.i, %bb47.i.i.i.i124.i.i.i.i ]
  %_38.i.i.i.i103.i.i.i.i = icmp ult i32 %r.sroa.0.025.i.i.i.i97.i.i.i.i, %w.sroa.0.1.i.i.i.i101.i.i.i.i
  %or.cond24.i.i.i.i104.i.i.i.i = select i1 %_1830.i.i.i.i92.i.i.i.i, i1 %_38.i.i.i.i103.i.i.i.i, i1 false
  br i1 %or.cond24.i.i.i.i104.i.i.i.i, label %bb21.i.i.i.i111.i.i.i.i, label %bb28.i.i.i.i105.i.i.i.i

bb11.i.i.i.i123.i.i.i.i:                          ; preds = %bb9.i.i.i.i91.i.i.i.i
  %175 = load volatile i32, ptr %158, align 4
  %176 = and i32 %175, 64
  %.not.i43 = icmp eq i32 %176, 0
  br i1 %.not.i43, label %177, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

177:                                              ; preds = %bb11.i.i.i.i123.i.i.i.i
  %178 = and i32 %175, 256
  %179 = icmp eq i32 %178, 0
  br i1 %179, label %180, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

180:                                              ; preds = %177
  %181 = and i32 %175, 32
  %182 = icmp eq i32 %181, 0
  br i1 %182, label %183, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

183:                                              ; preds = %180
  %184 = and i32 %175, 16
  %185 = icmp eq i32 %184, 0
  br i1 %185, label %bb47.i.i.i.i124.i.i.i.i, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

bb47.i.i.i.i124.i.i.i.i:                          ; preds = %183
  %186 = and i32 %175, 2
  %_82.i.i.i.i125.i.i.i.i.not = icmp eq i32 %186, 0
  br i1 %_82.i.i.i.i125.i.i.i.i.not, label %bb19.i.i.i.i100.i.i.i.i, label %bb13.i.i.i.i126.i.i.i.i

bb13.i.i.i.i126.i.i.i.i:                          ; preds = %bb47.i.i.i.i124.i.i.i.i
  %_92.i.i.i.i127.i.i.i.i = icmp eq ptr %write1.sroa.0.027.i.i.i.i95.i.i.i.i, %156
  br i1 %_92.i.i.i.i127.i.i.i.i, label %bb16.i.i.i.i131.i.i.i.i, label %bb52.i.i.i.i128.i.i.i.i

bb52.i.i.i.i128.i.i.i.i:                          ; preds = %bb13.i.i.i.i126.i.i.i.i
  %_98.i.i.i.i129.i.i.i.i = getelementptr inbounds nuw i8, ptr %write1.sroa.0.027.i.i.i.i95.i.i.i.i, i32 1
  %_33.i4.i.i.i130.i.i.i.i = load i8, ptr %write1.sroa.0.027.i.i.i.i95.i.i.i.i, align 1, !alias.scope !144, !noalias !145, !noundef !21
  br label %bb16.i.i.i.i131.i.i.i.i

bb16.i.i.i.i131.i.i.i.i:                          ; preds = %bb52.i.i.i.i128.i.i.i.i, %bb13.i.i.i.i126.i.i.i.i
  %.sink.i.i.i.i132.i.i.i.i = phi i8 [ %_33.i4.i.i.i130.i.i.i.i, %bb52.i.i.i.i128.i.i.i.i ], [ 0, %bb13.i.i.i.i126.i.i.i.i ]
  %write1.sroa.0.2.i.i.i.i133.i.i.i.i = phi ptr [ %_98.i.i.i.i129.i.i.i.i, %bb52.i.i.i.i128.i.i.i.i ], [ %156, %bb13.i.i.i.i126.i.i.i.i ]
  store volatile i8 %.sink.i.i.i.i132.i.i.i.i, ptr %_100.i.i.i.i90.i.i.i.i, align 4, !noalias !143
  %187 = add nuw nsw i32 %w.sroa.0.026.i.i.i.i96.i.i.i.i, 1
  br label %bb19.i.i.i.i100.i.i.i.i

bb28.i.i.i.i105.i.i.i.i:                          ; preds = %bb25.i.i.i.i119.i.i.i.i, %bb61.i.i.i.i112.i.i.i.i, %bb19.i.i.i.i100.i.i.i.i
  %r.sroa.0.1.i.i.i.i106.i.i.i.i = phi i32 [ %201, %bb25.i.i.i.i119.i.i.i.i ], [ %r.sroa.0.025.i.i.i.i97.i.i.i.i, %bb19.i.i.i.i100.i.i.i.i ], [ %r.sroa.0.025.i.i.i.i97.i.i.i.i, %bb61.i.i.i.i112.i.i.i.i ]
  %read2.sroa.0.1.i.i.i.i107.i.i.i.i = phi ptr [ %read2.sroa.0.2.i.i.i.i120.i.i.i.i, %bb25.i.i.i.i119.i.i.i.i ], [ %read2.sroa.0.028.i.i.i.i94.i.i.i.i, %bb19.i.i.i.i100.i.i.i.i ], [ %read2.sroa.0.028.i.i.i.i94.i.i.i.i, %bb61.i.i.i.i112.i.i.i.i ]
  %_16.i3.i.i.i108.i.i.i.i = icmp ult i32 %w.sroa.0.1.i.i.i.i101.i.i.i.i, %_17.1.i.i.i.i
  %_18.i.i.i.i109.i.i.i.i = icmp ult i32 %r.sroa.0.1.i.i.i.i106.i.i.i.i, %_17.1.i.i.i.i
  %or.cond.i.i.i.i110.i.i.i.i = select i1 %_16.i3.i.i.i108.i.i.i.i, i1 true, i1 %_18.i.i.i.i109.i.i.i.i
  br i1 %or.cond.i.i.i.i110.i.i.i.i, label %bb9.i.i.i.i91.i.i.i.i, label %bb1.i.i.backedge

bb21.i.i.i.i111.i.i.i.i:                          ; preds = %bb19.i.i.i.i100.i.i.i.i
  %188 = load volatile i32, ptr %158, align 4, !noalias !143
  %189 = and i32 %188, 64
  %190 = icmp eq i32 %189, 0
  br i1 %190, label %191, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

191:                                              ; preds = %bb21.i.i.i.i111.i.i.i.i
  %192 = and i32 %188, 256
  %193 = icmp eq i32 %192, 0
  br i1 %193, label %194, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

194:                                              ; preds = %191
  %195 = and i32 %188, 32
  %196 = icmp eq i32 %195, 0
  br i1 %196, label %197, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

197:                                              ; preds = %194
  %198 = and i32 %188, 16
  %199 = icmp eq i32 %198, 0
  br i1 %199, label %bb61.i.i.i.i112.i.i.i.i, label %_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit

bb61.i.i.i.i112.i.i.i.i:                          ; preds = %197
  %200 = trunc i32 %188 to i1
  br i1 %200, label %bb23.i.i.i.i114.i.i.i.i, label %bb28.i.i.i.i105.i.i.i.i

bb23.i.i.i.i114.i.i.i.i:                          ; preds = %bb61.i.i.i.i112.i.i.i.i
  %_120.i.i.i.i115.i.i.i.i = icmp eq ptr %read2.sroa.0.028.i.i.i.i94.i.i.i.i, %156
  br i1 %_120.i.i.i.i115.i.i.i.i, label %bb69.i.i.i.i121.i.i.i.i, label %bb67.i.i.i.i116.i.i.i.i

bb67.i.i.i.i116.i.i.i.i:                          ; preds = %bb23.i.i.i.i114.i.i.i.i
  %_126.i.i.i.i117.i.i.i.i = getelementptr inbounds nuw i8, ptr %read2.sroa.0.028.i.i.i.i94.i.i.i.i, i32 1
  %_47.i.i.i.i118.i.i.i.i = load volatile i8, ptr %_100.i.i.i.i90.i.i.i.i, align 4, !noalias !143
  store i8 %_47.i.i.i.i118.i.i.i.i, ptr %read2.sroa.0.028.i.i.i.i94.i.i.i.i, align 1, !alias.scope !144, !noalias !145
  br label %bb25.i.i.i.i119.i.i.i.i

bb25.i.i.i.i119.i.i.i.i:                          ; preds = %bb69.i.i.i.i121.i.i.i.i, %bb67.i.i.i.i116.i.i.i.i
  %read2.sroa.0.2.i.i.i.i120.i.i.i.i = phi ptr [ %156, %bb69.i.i.i.i121.i.i.i.i ], [ %_126.i.i.i.i117.i.i.i.i, %bb67.i.i.i.i116.i.i.i.i ]
  %201 = add nuw nsw i32 %r.sroa.0.025.i.i.i.i97.i.i.i.i, 1
  br label %bb28.i.i.i.i105.i.i.i.i

bb69.i.i.i.i121.i.i.i.i:                          ; preds = %bb23.i.i.i.i114.i.i.i.i
  %_49.i.i.i.i122.i.i.i.i = load volatile i8, ptr %_100.i.i.i.i90.i.i.i.i, align 4, !noalias !143
  br label %bb25.i.i.i.i119.i.i.i.i

bb8.i.i.i81.i.i.i.i:                              ; preds = %bb8.i.i.i81.i.i.i.i.3, %bb8.lr.ph.i.i.i79.i.i.i.i
  %_23.i.i.i82.i.i.i.i = load volatile i32, ptr %_29.i.i.i80.i.i.i.i, align 4, !noalias !143
  %_22.i.i.i.i.i.i.i = load volatile i32, ptr %_24.i.i.i78.i.i.i.i, align 4, !noalias !143
  %_27.i.i.i83.i.i.i.i = and i32 %_22.i.i.i.i.i.i.i, 1
  %_21.not.i.i.i.i.i.i.i = icmp eq i32 %_27.i.i.i83.i.i.i.i, 0
  br i1 %_21.not.i.i.i.i.i.i.i, label %bb9.i.i.i84.i.i.i.i, label %bb8.i.i.i81.i.i.i.i.1

bb8.i.i.i81.i.i.i.i.1:                            ; preds = %bb8.i.i.i81.i.i.i.i
  %_23.i.i.i82.i.i.i.i.1 = load volatile i32, ptr %_29.i.i.i80.i.i.i.i, align 4, !noalias !143
  %_22.i.i.i.i.i.i.i.1 = load volatile i32, ptr %_24.i.i.i78.i.i.i.i, align 4, !noalias !143
  %_27.i.i.i83.i.i.i.i.1 = and i32 %_22.i.i.i.i.i.i.i.1, 1
  %_21.not.i.i.i.i.i.i.i.1 = icmp eq i32 %_27.i.i.i83.i.i.i.i.1, 0
  br i1 %_21.not.i.i.i.i.i.i.i.1, label %bb9.i.i.i84.i.i.i.i, label %bb8.i.i.i81.i.i.i.i.2

bb8.i.i.i81.i.i.i.i.2:                            ; preds = %bb8.i.i.i81.i.i.i.i.1
  %_23.i.i.i82.i.i.i.i.2 = load volatile i32, ptr %_29.i.i.i80.i.i.i.i, align 4, !noalias !143
  %_22.i.i.i.i.i.i.i.2 = load volatile i32, ptr %_24.i.i.i78.i.i.i.i, align 4, !noalias !143
  %_27.i.i.i83.i.i.i.i.2 = and i32 %_22.i.i.i.i.i.i.i.2, 1
  %_21.not.i.i.i.i.i.i.i.2 = icmp eq i32 %_27.i.i.i83.i.i.i.i.2, 0
  br i1 %_21.not.i.i.i.i.i.i.i.2, label %bb9.i.i.i84.i.i.i.i, label %bb8.i.i.i81.i.i.i.i.3

bb8.i.i.i81.i.i.i.i.3:                            ; preds = %bb8.i.i.i81.i.i.i.i.2
  %_23.i.i.i82.i.i.i.i.3 = load volatile i32, ptr %_29.i.i.i80.i.i.i.i, align 4, !noalias !143
  %_22.i.i.i.i.i.i.i.3 = load volatile i32, ptr %_24.i.i.i78.i.i.i.i, align 4, !noalias !143
  %_27.i.i.i83.i.i.i.i.3 = and i32 %_22.i.i.i.i.i.i.i.3, 1
  %_21.not.i.i.i.i.i.i.i.3 = icmp eq i32 %_27.i.i.i83.i.i.i.i.3, 0
  br i1 %_21.not.i.i.i.i.i.i.i.3, label %bb9.i.i.i84.i.i.i.i, label %bb8.i.i.i81.i.i.i.i

bb2.i.i.i.i:                                      ; preds = %bb3.i.i
; call embedded_hal_bus::spi::no_delay_panic
  tail call fastcc void @_RNvNtCsgYNjhtQEXp_16embedded_hal_bus3spi14no_delay_panic() #3, !noalias !146
  unreachable

_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression.exit: ; preds = %bb1.i.i, %159, %169, %163, %166, %194, %191, %197, %bb21.i.i.i.i111.i.i.i.i, %180, %177, %183, %bb11.i.i.i.i123.i.i.i.i, %106, %116, %110, %113, %141, %138, %144, %bb21.i.i.i.i52.i.i.i.i, %127, %124, %130, %bb11.i.i.i.i64.i.i.i.i, %12, %22, %16, %19, %47, %44, %50, %bb21.i.i.i.i.i.i.i.i, %33, %30, %36, %bb11.i.i.i.i.i.i.i.i, %63, %73, %67, %70, %79, %89, %83, %86
  %_0.sroa.0.0.i.i = phi i8 [ 2, %19 ], [ 3, %159 ], [ 0, %110 ], [ 3, %bb21.i.i.i.i111.i.i.i.i ], [ 3, %bb21.i.i.i.i.i.i.i.i ], [ 3, %bb11.i.i.i.i64.i.i.i.i ], [ 2, %70 ], [ 2, %86 ], [ 3, %79 ], [ 1, %89 ], [ 0, %83 ], [ 3, %63 ], [ 1, %73 ], [ 0, %67 ], [ 1, %50 ], [ 0, %44 ], [ 2, %47 ], [ 1, %36 ], [ 0, %30 ], [ 2, %33 ], [ 3, %bb11.i.i.i.i.i.i.i.i ], [ 3, %12 ], [ 1, %22 ], [ 0, %16 ], [ 1, %144 ], [ 0, %138 ], [ 2, %141 ], [ 1, %130 ], [ 0, %124 ], [ 2, %127 ], [ 3, %bb21.i.i.i.i52.i.i.i.i ], [ 2, %113 ], [ 3, %106 ], [ 1, %116 ], [ 2, %180 ], [ 3, %bb11.i.i.i.i123.i.i.i.i ], [ 1, %197 ], [ 2, %194 ], [ 1, %183 ], [ 0, %177 ], [ 0, %191 ], [ 1, %169 ], [ 0, %163 ], [ 2, %166 ], [ -1, %bb1.i.i ]
  tail call void @llvm.experimental.noalias.scope.decl(metadata !147)
  tail call void @llvm.experimental.noalias.scope.decl(metadata !150)
  %self.val.i.i.i7.i = load i8, ptr %_2.i.i, align 1, !alias.scope !153, !noalias !154, !noundef !21
  %_0.i.i.i.i8.i = and i8 %self.val.i.i.i7.i, 15
  %_01.i.i.i.i.i9.i = lshr i8 %self.val.i.i.i7.i, 4
  %_2.i.i.i.i10.i = zext nneg i8 %_01.i.i.i.i.i9.i to i32
  %_6.i.i.i.i11.i = shl nuw nsw i32 %_2.i.i.i.i10.i, 10
  %_5.i.i.i.i12.i = or disjoint i32 %_6.i.i.i.i11.i, 1073872896
  %_4.i.i.i.i13.i = inttoptr i32 %_5.i.i.i.i12.i to ptr
  %_6.i.i.i14.i = getelementptr i8, ptr %_4.i.i.i.i13.i, i32 24
  %n.i.i.i.i = zext nneg i8 %_0.i.i.i.i8.i to i32
  %_18.i.i.i.i = shl nuw nsw i32 1, %n.i.i.i.i
  store volatile i32 %_18.i.i.i.i, ptr %_6.i.i.i14.i, align 8, !noalias !157
  call void @llvm.lifetime.end.p0(ptr nonnull %_8)
  ret i8 %_0.sroa.0.0.i.i
}

; Function Attrs: mustprogress nocallback nofree nosync nounwind willreturn memory(argmem: readwrite)
declare void @llvm.lifetime.start.p0(ptr captures(none)) #1

; Function Attrs: mustprogress nocallback nofree nosync nounwind willreturn memory(argmem: readwrite)
declare void @llvm.lifetime.end.p0(ptr captures(none)) #1

; Function Attrs: mustprogress nocallback nocreateundeforpoison nofree nosync nounwind speculatable willreturn memory(none)
declare i32 @llvm.umax.i32(i32, i32) #2

; Function Attrs: nounwind
declare void @llvm.arm.hint(i32) #3

; Function Attrs: mustprogress nocallback nofree nosync nounwind willreturn memory(inaccessiblemem: readwrite)
declare void @llvm.experimental.noalias.scope.decl(metadata) #4

; core::panicking::panic_fmt
; Function Attrs: cold noinline noreturn nounwind
define internal fastcc void @_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt() unnamed_addr #5 !dbg !158 !guid !163 {
  br label %bb1.i, !dbg !164

bb1.i:                                            ; preds = %bb1.i, %0
  tail call void @llvm.arm.hint(i32 1), !dbg !164
  tail call void @llvm.arm.hint(i32 1), !dbg !164
  tail call void @llvm.arm.hint(i32 1), !dbg !164
  tail call void @llvm.arm.hint(i32 1), !dbg !164
  br label %bb1.i, !dbg !164
}

; embedded_hal_bus::spi::no_delay_panic
; Function Attrs: cold noreturn nounwind
define internal fastcc void @_RNvNtCsgYNjhtQEXp_16embedded_hal_bus3spi14no_delay_panic() unnamed_addr #6 !guid !166 {
; call core::panicking::panic_fmt
  tail call fastcc void @_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt() #7
  unreachable
}

attributes #0 = { nounwind "frame-pointer"="all" "target-cpu"="generic" "target-features"="+vfp4d16sp,+fpregs,+vfp2sp" }
attributes #1 = { mustprogress nocallback nofree nosync nounwind willreturn memory(argmem: readwrite) }
attributes #2 = { mustprogress nocallback nocreateundeforpoison nofree nosync nounwind speculatable willreturn memory(none) }
attributes #3 = { nounwind }
attributes #4 = { mustprogress nocallback nofree nosync nounwind willreturn memory(inaccessiblemem: readwrite) }
attributes #5 = { cold noinline noreturn nounwind "frame-pointer"="all" "target-cpu"="generic" "target-features"="+vfp4d16sp" }
attributes #6 = { cold noreturn nounwind "frame-pointer"="all" "target-cpu"="generic" "target-features"="+vfp4d16sp,+fpregs,+vfp2sp" }
attributes #7 = { noreturn nounwind }

!llvm.module.flags = !{!2, !3, !4}
!llvm.ident = !{!5}
!llvm.dbg.cu = !{!6}

!0 = !{i64 -5654952517103725453}
!1 = !{i64 -2401422355947093301}
!2 = !{i32 7, !"frame-pointer", i32 2}
!3 = !{i32 7, !"Dwarf Version", i32 4}
!4 = !{i32 2, !"Debug Info Version", i32 3}
!5 = !{!"rustc version 1.101.0-nightly (822182db4 2026-10-13)"}
!6 = distinct !DICompileUnit(language: DW_LANG_Rust, file: !7, producer: "clang LLVM (rustc version 1.101.0-nightly (822182db4 2026-10-13))", isOptimized: true, runtimeVersion: 0, emissionKind: FullDebug, splitDebugInlining: false, nameTableKind: None)
!7 = !DIFile(filename: "/rustc/822182db4c4a1827256881d8c7aa685f0bcf2b11/library/core/src/lib.rs/@/core.14fadb22da8e1282-cgu.0", directory: "/rustc/822182db4c4a1827256881d8c7aa685f0bcf2b11")
!8 = !{i64 935730666717439989}
!9 = !{!10}
!10 = distinct !{!10, !11, !"_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression: %operations.0"}
!11 = distinct !{!11, !"_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression"}
!12 = !{!13}
!13 = distinct !{!13, !11, !"_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression: %bus"}
!14 = !{!15}
!15 = distinct !{!15, !11, !"_RINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB16_4mode8BlockingNtNtB14_4mode6MasterEQNtNtB16_4gpio6OutputNtB4_7NoDelayECs28Up07JOROr_14spi_regression: %cs"}
!16 = !{!17}
!17 = distinct !{!17, !18, !"_RNvXs6_NtCshmzHxTFkRd_12embedded_hal7digitalQNtNtCsbZ84FdmNbEi_13embassy_stm324gpio6OutputNtB5_9OutputPin7set_lowCs28Up07JOROr_14spi_regression: %self"}
!18 = distinct !{!18, !"_RNvXs6_NtCshmzHxTFkRd_12embedded_hal7digitalQNtNtCsbZ84FdmNbEi_13embassy_stm324gpio6OutputNtB5_9OutputPin7set_lowCs28Up07JOROr_14spi_regression"}
!19 = !{!17, !15}
!20 = !{!10, !13}
!21 = !{}
!22 = !{!23}
!23 = distinct !{!23, !24, !"_RNvXss_NtCsbZ84FdmNbEi_13embassy_stm324gpioNtB5_6OutputNtNtCshmzHxTFkRd_12embedded_hal7digital9OutputPin7set_low: %self"}
!24 = distinct !{!24, !"_RNvXss_NtCsbZ84FdmNbEi_13embassy_stm324gpioNtB5_6OutputNtNtCshmzHxTFkRd_12embedded_hal7digital9OutputPin7set_low"}
!25 = !{!26}
!26 = distinct !{!26, !27, !"_RNvYNtNtCsbZ84FdmNbEi_13embassy_stm324gpio6AnyPinNtB4_9SealedPin7set_lowCs28Up07JOROr_14spi_regression: %self"}
!27 = distinct !{!27, !"_RNvYNtNtCsbZ84FdmNbEi_13embassy_stm324gpio6AnyPinNtB4_9SealedPin7set_lowCs28Up07JOROr_14spi_regression"}
!28 = !{!26, !23}
!29 = !{!17, !10, !13, !15}
!30 = !{!26, !23, !17, !10, !13, !15}
!31 = !{!10, !15}
!32 = !{i64 4}
!33 = !{!34}
!34 = distinct !{!34, !35, !"_RNCINvNvNtNtNtNtCs1NFSOanQ2Tm_4core4iter6traits8iterator8Iterator12try_for_each4callQINtNtCshmzHxTFkRd_12embedded_hal3spi9OperationhEINtNtBe_6result6ResultuNtNtCsbZ84FdmNbEi_13embassy_stm323spi5ErrorENCINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtB2w_3SpiNtNtB2y_4mode8BlockingNtNtB2w_4mode6MasterEQNtNtB2y_4gpio6OutputNtB3j_7NoDelayE0E0Cs28Up07JOROr_14spi_regression: %x"}
!35 = distinct !{!35, !"_RNCINvNvNtNtNtNtCs1NFSOanQ2Tm_4core4iter6traits8iterator8Iterator12try_for_each4callQINtNtCshmzHxTFkRd_12embedded_hal3spi9OperationhEINtNtBe_6result6ResultuNtNtCsbZ84FdmNbEi_13embassy_stm323spi5ErrorENCINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtB2w_3SpiNtNtB2y_4mode8BlockingNtNtB2w_4mode6MasterEQNtNtB2y_4gpio6OutputNtB3j_7NoDelayE0E0Cs28Up07JOROr_14spi_regression"}
!36 = !{!37}
!37 = distinct !{!37, !38, !"_RNCINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB18_4mode8BlockingNtNtB16_4mode6MasterEQNtNtB18_4gpio6OutputNtB6_7NoDelayE0Cs28Up07JOROr_14spi_regression: %op"}
!38 = distinct !{!38, !"_RNCINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtB18_4mode8BlockingNtNtB16_4mode6MasterEQNtNtB18_4gpio6OutputNtB6_7NoDelayE0Cs28Up07JOROr_14spi_regression"}
!39 = !{i32 0, i32 5}
!40 = !{!37, !34, !10}
!41 = !{!42, !44, !13, !15}
!42 = distinct !{!42, !43, !"_RINvYINtNtNtCs1NFSOanQ2Tm_4core5slice4iter7IterMutINtNtCshmzHxTFkRd_12embedded_hal3spi9OperationhEENtNtNtNtBa_4iter6traits8iterator8Iterator8try_folduNCINvNvB1z_12try_for_each4callQBM_INtNtBa_6result6ResultuNtNtCsbZ84FdmNbEi_13embassy_stm323spi5ErrorENCINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtB3l_3SpiNtNtB3n_4mode8BlockingNtNtB3l_4mode6MasterEQNtNtB3n_4gpio6OutputNtB48_7NoDelayE0E0B2W_ECs28Up07JOROr_14spi_regression: %self"}
!43 = distinct !{!43, !"_RINvYINtNtNtCs1NFSOanQ2Tm_4core5slice4iter7IterMutINtNtCshmzHxTFkRd_12embedded_hal3spi9OperationhEENtNtNtNtBa_4iter6traits8iterator8Iterator8try_folduNCINvNvB1z_12try_for_each4callQBM_INtNtBa_6result6ResultuNtNtCsbZ84FdmNbEi_13embassy_stm323spi5ErrorENCINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtB3l_3SpiNtNtB3n_4mode8BlockingNtNtB3l_4mode6MasterEQNtNtB3n_4gpio6OutputNtB48_7NoDelayE0E0B2W_ECs28Up07JOROr_14spi_regression"}
!44 = distinct !{!44, !43, !"_RINvYINtNtNtCs1NFSOanQ2Tm_4core5slice4iter7IterMutINtNtCshmzHxTFkRd_12embedded_hal3spi9OperationhEENtNtNtNtBa_4iter6traits8iterator8Iterator8try_folduNCINvNvB1z_12try_for_each4callQBM_INtNtBa_6result6ResultuNtNtCsbZ84FdmNbEi_13embassy_stm323spi5ErrorENCINvNtNtCsgYNjhtQEXp_16embedded_hal_bus3spi6shared11transactionhQINtB3l_3SpiNtNtB3n_4mode8BlockingNtNtB3l_4mode6MasterEQNtNtB3n_4gpio6OutputNtB48_7NoDelayE0E0B2W_ECs28Up07JOROr_14spi_regression: argument 1"}
!45 = !{!46}
!46 = distinct !{!46, !47, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus4readCs28Up07JOROr_14spi_regression: %words.0"}
!47 = distinct !{!47, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus4readCs28Up07JOROr_14spi_regression"}
!48 = !{!49}
!49 = distinct !{!49, !50, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus4readCs28Up07JOROr_14spi_regression: %self"}
!50 = distinct !{!50, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus4readCs28Up07JOROr_14spi_regression"}
!51 = !{!52}
!52 = distinct !{!52, !50, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus4readCs28Up07JOROr_14spi_regression: %words.0"}
!53 = !{!54}
!54 = distinct !{!54, !55, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE13blocking_readhECs28Up07JOROr_14spi_regression: %self"}
!55 = distinct !{!55, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE13blocking_readhECs28Up07JOROr_14spi_regression"}
!56 = !{!57}
!57 = distinct !{!57, !55, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE13blocking_readhECs28Up07JOROr_14spi_regression: %words.0"}
!58 = !{!59}
!59 = distinct !{!59, !60, !"_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression: %self"}
!60 = distinct !{!60, !"_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression"}
!61 = !{i8 0, i8 2}
!62 = !{!59, !54, !49}
!63 = !{!57, !52, !46, !37, !34, !42, !44, !10, !13, !15}
!64 = !{!54, !49}
!65 = !{!59, !54, !57, !49, !52, !46, !37, !34, !42, !44, !10, !13, !15}
!66 = !{!54, !57, !49, !52, !46, !37, !34, !42, !44, !10, !13, !15}
!67 = !{!57, !52, !46}
!68 = !{!54, !49, !37, !34, !42, !44, !10, !13, !15}
!69 = !{!70}
!70 = distinct !{!70, !71, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus5writeCs28Up07JOROr_14spi_regression: %words.0"}
!71 = distinct !{!71, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus5writeCs28Up07JOROr_14spi_regression"}
!72 = !{!73}
!73 = distinct !{!73, !74, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus5writeCs28Up07JOROr_14spi_regression: %self"}
!74 = distinct !{!74, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus5writeCs28Up07JOROr_14spi_regression"}
!75 = !{!76}
!76 = distinct !{!76, !74, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus5writeCs28Up07JOROr_14spi_regression: %words.0"}
!77 = !{!78}
!78 = distinct !{!78, !79, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE14blocking_writehECs28Up07JOROr_14spi_regression: %self"}
!79 = distinct !{!79, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE14blocking_writehECs28Up07JOROr_14spi_regression"}
!80 = !{!81}
!81 = distinct !{!81, !79, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE14blocking_writehECs28Up07JOROr_14spi_regression: %words.0"}
!82 = !{!83}
!83 = distinct !{!83, !84, !"_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression: %self"}
!84 = distinct !{!84, !"_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression"}
!85 = !{!83, !78, !73}
!86 = !{!81, !76, !70, !37, !34, !42, !44, !10, !13, !15}
!87 = !{!78, !73}
!88 = !{!83, !78, !81, !73, !76, !70, !37, !34, !42, !44, !10, !13, !15}
!89 = !{!78, !81, !73, !76, !70, !37, !34, !42, !44, !10, !13, !15}
!90 = !{!81, !76, !70}
!91 = !{!78, !73, !37, !34, !42, !44, !10, !13, !15}
!92 = !{!93}
!93 = distinct !{!93, !94, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus8transferCs28Up07JOROr_14spi_regression: %read.0"}
!94 = distinct !{!94, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus8transferCs28Up07JOROr_14spi_regression"}
!95 = !{!96}
!96 = distinct !{!96, !94, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus8transferCs28Up07JOROr_14spi_regression: %write.0"}
!97 = !{!98}
!98 = distinct !{!98, !99, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus8transferCs28Up07JOROr_14spi_regression: %self"}
!99 = distinct !{!99, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus8transferCs28Up07JOROr_14spi_regression"}
!100 = !{!101}
!101 = distinct !{!101, !99, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus8transferCs28Up07JOROr_14spi_regression: %read.0"}
!102 = !{!103}
!103 = distinct !{!103, !99, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus8transferCs28Up07JOROr_14spi_regression: %write.0"}
!104 = !{!105}
!105 = distinct !{!105, !106, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE17blocking_transferhECs28Up07JOROr_14spi_regression: %self"}
!106 = distinct !{!106, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE17blocking_transferhECs28Up07JOROr_14spi_regression"}
!107 = !{!108}
!108 = distinct !{!108, !106, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE17blocking_transferhECs28Up07JOROr_14spi_regression: %read.0"}
!109 = !{!110}
!110 = distinct !{!110, !106, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE17blocking_transferhECs28Up07JOROr_14spi_regression: %write.0"}
!111 = !{!112}
!112 = distinct !{!112, !113, !"_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression: %self"}
!113 = distinct !{!113, !"_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression"}
!114 = !{!112, !105, !98}
!115 = !{!108, !110, !101, !103, !93, !96, !37, !34, !42, !44, !10, !13, !15}
!116 = !{!105, !98}
!117 = !{!112, !105, !108, !110, !98, !101, !103, !93, !96, !37, !34, !42, !44, !10, !13, !15}
!118 = !{!105, !108, !110, !98, !101, !103, !93, !96, !37, !34, !42, !44, !10, !13, !15}
!119 = !{!110, !103, !96}
!120 = !{!105, !108, !98, !101, !93, !37, !34, !42, !44, !10, !13, !15}
!121 = !{!108, !101, !93}
!122 = !{!105, !110, !98, !103, !96, !37, !34, !42, !44, !10, !13, !15}
!123 = !{!124}
!124 = distinct !{!124, !125, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus17transfer_in_placeCs28Up07JOROr_14spi_regression: %words.0"}
!125 = distinct !{!125, !"_RNvXs3_NtCshmzHxTFkRd_12embedded_hal3spiQINtNtCsbZ84FdmNbEi_13embassy_stm323spi3SpiNtNtBI_4mode8BlockingNtNtBG_4mode6MasterENtB5_6SpiBus17transfer_in_placeCs28Up07JOROr_14spi_regression"}
!126 = !{!127}
!127 = distinct !{!127, !128, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus17transfer_in_placeCs28Up07JOROr_14spi_regression: %self"}
!128 = distinct !{!128, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus17transfer_in_placeCs28Up07JOROr_14spi_regression"}
!129 = !{!130}
!130 = distinct !{!130, !128, !"_RNvXsc_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterENtNtCshmzHxTFkRd_12embedded_hal3spi6SpiBus17transfer_in_placeCs28Up07JOROr_14spi_regression: %words.0"}
!131 = !{!132}
!132 = distinct !{!132, !133, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE26blocking_transfer_in_placehECs28Up07JOROr_14spi_regression: %self"}
!133 = distinct !{!133, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE26blocking_transfer_in_placehECs28Up07JOROr_14spi_regression"}
!134 = !{!135}
!135 = distinct !{!135, !133, !"_RINvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB6_3SpiNtNtB8_4mode8BlockingNtNtB6_4mode6MasterE26blocking_transfer_in_placehECs28Up07JOROr_14spi_regression: %words.0"}
!136 = !{!137}
!137 = distinct !{!137, !138, !"_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression: %self"}
!138 = distinct !{!138, !"_RNvMs3_NtCsbZ84FdmNbEi_13embassy_stm323spiINtB5_3SpiNtNtB7_4mode8BlockingNtNtB5_4mode6MasterE13set_word_sizeCs28Up07JOROr_14spi_regression"}
!139 = !{!137, !132, !127}
!140 = !{!135, !130, !124, !37, !34, !42, !44, !10, !13, !15}
!141 = !{!132, !127}
!142 = !{!137, !132, !135, !127, !130, !124, !37, !34, !42, !44, !10, !13, !15}
!143 = !{!132, !135, !127, !130, !124, !37, !34, !42, !44, !10, !13, !15}
!144 = !{!135, !130, !124}
!145 = !{!132, !127, !37, !34, !42, !44, !10, !13, !15}
!146 = !{!37, !34, !42, !44, !10, !13, !15}
!147 = !{!148}
!148 = distinct !{!148, !149, !"_RNvXss_NtCsbZ84FdmNbEi_13embassy_stm324gpioNtB5_6OutputNtNtCshmzHxTFkRd_12embedded_hal7digital9OutputPin8set_high: %self"}
!149 = distinct !{!149, !"_RNvXss_NtCsbZ84FdmNbEi_13embassy_stm324gpioNtB5_6OutputNtNtCshmzHxTFkRd_12embedded_hal7digital9OutputPin8set_high"}
!150 = !{!151}
!151 = distinct !{!151, !152, !"_RNvYNtNtCsbZ84FdmNbEi_13embassy_stm324gpio6AnyPinNtB4_9SealedPin8set_highCs28Up07JOROr_14spi_regression: %self"}
!152 = distinct !{!152, !"_RNvYNtNtCsbZ84FdmNbEi_13embassy_stm324gpio6AnyPinNtB4_9SealedPin8set_highCs28Up07JOROr_14spi_regression"}
!153 = !{!151, !148}
!154 = !{!155, !10, !13, !15}
!155 = distinct !{!155, !156, !"_RNvXs6_NtCshmzHxTFkRd_12embedded_hal7digitalQNtNtCsbZ84FdmNbEi_13embassy_stm324gpio6OutputNtB5_9OutputPin8set_highCs28Up07JOROr_14spi_regression: %self"}
!156 = distinct !{!156, !"_RNvXs6_NtCshmzHxTFkRd_12embedded_hal7digitalQNtNtCsbZ84FdmNbEi_13embassy_stm324gpio6OutputNtB5_9OutputPin8set_highCs28Up07JOROr_14spi_regression"}
!157 = !{!151, !148, !155, !10, !13, !15}
!158 = distinct !DISubprogram(name: "panic_fmt", linkageName: "_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt", scope: !160, file: !159, line: 60, type: !162, scopeLine: 60, flags: DIFlagPrototyped | DIFlagNoReturn, spFlags: DISPFlagDefinition | DISPFlagOptimized, unit: !6, templateParams: !21)
!159 = !DIFile(filename: "library/core/src/panicking.rs", directory: "/rustc/822182db4c4a1827256881d8c7aa685f0bcf2b11", checksumkind: CSK_MD5, checksum: "c708a818a9762f274b6effcc11ac5617")
!160 = !DINamespace(name: "panicking", scope: !161)
!161 = !DINamespace(name: "core", scope: null)
!162 = !DISubroutineType(cc: DW_CC_nocall, types: !21)
!163 = !{i64 -4119270582606525714}
!164 = !DILocation(line: 80, column: 14, scope: !165)
!165 = distinct !DILexicalBlock(scope: !158, file: !159, line: 72, column: 5)
!166 = !{i64 7381553805052548788}
