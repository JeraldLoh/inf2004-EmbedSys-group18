#include "wheel_measurement.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static wheel_measurement_t m;
static motion_encoder_snapshot_t s;
static void init(bool bounded) {
    wheel_measurement_config_t c={65U,{1271U,1264U},100000U,200000U,20000U,0U,0U};
    if(bounded) { c.max_batched_edge_permille=50U; c.max_batches_per_100ms=2U; }
    assert(wheel_measurement_init(&m,&c));
    memset(&s,0,sizeof(s)); s.acquisition_valid=true; s.edge_times_valid=true;
    s.command_basis_valid=true; s.left_command_sign=1; s.right_command_sign=1; s.reset_epoch=1U;
    assert(wheel_measurement_update(&m,&s)==MOTION_OK);
}
static motion_status_t tick(unsigned edges,unsigned left,unsigned right) {
    s.acquired_us+=20000U; s.left_count+=edges; s.right_count+=edges;
    s.coalesced_batches[0]+=left; s.coalesced_batches[1]+=right;
    s.coalesced_edges_seen|=left!=0U || right!=0U;
    return wheel_measurement_update(&m,&s);
}
static void clean_window(void) {
    unsigned i; for(i=0;i<5U;++i) { assert(tick(12U,0U,0U)==MOTION_OK); }
    assert(m.sample.speed_valid && m.sample.speed_um_s[0]>90000U && m.sample.speed_um_s[0]<110000U);
}
int main(int argc,char **argv) {
    unsigned i; assert(argc==2);
    if(strcmp(argv[1],"strict")==0) {
        init(false); clean_window(); assert(tick(12U,1U,0U)==MOTION_UNAVAILABLE);
        assert(!m.sample.speed_valid); clean_window(); /* History stays visible. */
        assert(s.coalesced_edges_seen && m.sample.speed_valid);
    } else if(strcmp(argv[1],"sparse")==0) {
        init(true);
        for(i=0;i<5U;++i) { assert(tick(12U,i==2U?1U:0U,0U)==MOTION_OK); }
        assert(m.sample.speed_valid && !m.sample.signed_speed_valid && m.sample.window_batches[0]==1U);
        assert(m.sample.travel_um[0]==60ULL*65U*3141593U/(1271U*1000U));
        clean_window(); assert(m.sample.signed_speed_valid);
    } else if(strcmp(argv[1],"dense")==0) {
        init(true); assert(tick(20U,1U,0U)==MOTION_OK); assert(tick(20U,1U,0U)==MOTION_OK);
        assert(tick(20U,1U,0U)==MOTION_UNAVAILABLE); assert(!m.sample.speed_valid);
        clean_window();
    } else if(strcmp(argv[1],"fraction")==0) {
        init(true);
        for(i=0;i<4U;++i) { assert(tick(2U,i==2U?1U:0U,0U)==MOTION_OK); }
        assert(tick(2U,0U,0U)==MOTION_UNAVAILABLE); /* 2/10=20%, above5%. */
        clean_window();
    } else if(strcmp(argv[1],"independent")==0) {
        init(true);
        for(i=0;i<5U;++i) { assert(tick(20U,i==2U?2U:0U,i==3U?2U:0U)==MOTION_OK); }
        assert(m.sample.speed_valid && m.sample.window_batches[0]==2U && m.sample.window_batches[1]==2U);
    } else if(strcmp(argv[1],"recovery")==0) {
        init(true); clean_window(); s.acquired_us+=300000U;
        assert(wheel_measurement_update(&m,&s)==MOTION_UNAVAILABLE);
        for(i=0;i<4U;++i) { assert(tick(12U,0U,0U)==MOTION_OK); assert(!m.sample.speed_valid); }
        assert(tick(12U,0U,0U)==MOTION_OK && m.sample.speed_valid);
        ++s.reset_epoch; s.acquired_us+=20000U;
        assert(wheel_measurement_update(&m,&s)==MOTION_UNAVAILABLE); clean_window();
    } else if(strcmp(argv[1],"wrap")==0) {
        init(true); s.coalesced_batches[0]=UINT32_MAX; s.left_count=UINT32_MAX-5U;
        s.acquired_us=UINT32_MAX-50000U; m.primed=false;
        assert(wheel_measurement_update(&m,&s)==MOTION_OK);
        for(i=0;i<5U;++i) { assert(tick(12U,i==2U?1U:0U,0U)==MOTION_OK); }
        assert(s.coalesced_batches[0]==0U && m.sample.window_batches[0]==1U && m.sample.speed_valid);
    } else if(strcmp(argv[1],"invalid")==0) {
        init(true); s.acquired_us+=20000U; ++s.coalesced_batches[0]; ++s.left_count;
        assert(wheel_measurement_update(&m,&s)==MOTION_UNAVAILABLE); /* Cannot have2 batch edges in1 count. */
        clean_window(); s.acquisition_valid=false;
        assert(wheel_measurement_update(&m,&s)==MOTION_UNAVAILABLE && !m.primed);
    } else { assert(false); }
    printf("PASS encoder quality %s\n",argv[1]); return 0;
}
