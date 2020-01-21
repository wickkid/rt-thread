#include <udp_ethernet.h>

#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

#pragma pack(1)
typedef struct {
	uint16_t version;
	uint16_t type;
	uint32_t stNo;
	uint32_t sqNo;	
	uint32_t total;
	uint32_t position;
}eth_udp_hdr_t;
typedef struct {
	udp_list_t						list;
	eth_udp_hdr_t 			header;
	uint32_t						payloadLength;
	uint8_t							payload[UDP_ETH_MAXPKTSIZE];
}eth_udp_pack_t;
#pragma pack()

typedef struct{
	uint16_t 						version;
	uint32_t 						r_no;
	uint32_t 						r_sq;
	uint32_t 						s_no;
	int			 						sock;
	struct sockaddr_in	local;
	struct sockaddr_in	remote;
	udp_mutex_t 				mutex;
	udp_mp_t							s_mempool;
	udp_mp_t							r_mempool;
	udp_list_t						list;
	udp_list_t						r_list;
	udp_sem_t							r_sem;
	udp_list_t						s_list;
	udp_sem_t							s_sem;
	udp_list_t						w_list;
	
	void*									user_data;
}eth_udp_handler_t;

void eth_api_free_list(udp_list_t* header){
	if (!header) return;
	if (udp_list_isempty(header)) return;
	while (header->prev != header){
		udp_list_t* prev = header->prev;
		eth_udp_pack_t* pack = (eth_udp_pack_t*)udp_list_entry(prev, eth_udp_pack_t, list);
		udp_list_remove(&pack->list);
		udp_mp_free(pack);
	}
}


udp_eth_api_handler_t udp_eth_create(uint16_t localport, const char* remoteip, uint16_t remoteport, const char* name){
	eth_udp_handler_t* handler = (eth_udp_handler_t*)udp_malloc(sizeof(eth_udp_handler_t));
	int opt = 1;
	int sz = 0;
	char namebuf[16];
	assert(handler);
	udp_memset(handler, 0, sizeof(eth_udp_handler_t));
	handler->version = UDP_ETH_VERSION;
	
	handler->sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (-1 == handler->sock){
		udp_free(handler);
		return NULL;
	}
	sz = udp_snprintf(namebuf, 4, "%s", name);
	handler->local.sin_family = AF_INET;
  handler->local.sin_port = htons(localport);
	handler->local.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("192.168.3.199");//htonl(INADDR_ANY)
	
	bind(handler->sock, (struct sockaddr *) &handler->local, sizeof(handler->local));
	setsockopt (handler->sock, SOL_SOCKET, SO_BROADCAST, (char *) & opt, sizeof (int));

	udp_snprintf(namebuf+sz, 16 - sz, "_mt");
	handler->mutex = udp_mutex_create(namebuf);
	assert(handler->mutex);
	udp_snprintf(namebuf+sz, 16 - sz, "_rs");
	handler->r_sem = udp_sem_create(namebuf);
	assert(handler->r_sem);
	udp_snprintf(namebuf+sz, 16 - sz, "_ss");
	handler->s_sem = udp_sem_create(namebuf);
	assert(handler->s_sem);
	
	udp_snprintf(namebuf+sz, 16 - sz, "01");
	handler->s_mempool = udp_mp_create(namebuf, UPD_ETH_MAKPKTCOUNT, sizeof(eth_udp_pack_t));
	assert(handler->s_mempool);
	udp_snprintf(namebuf+sz, 16 - sz, "02");
	handler->r_mempool = udp_mp_create(namebuf, UPD_ETH_MAKPKTCOUNT, sizeof(eth_udp_pack_t));
	assert(handler->r_mempool);	
	
	udp_list_init(&handler->list);
	udp_list_init(&handler->r_list);
	udp_list_init(&handler->s_list);
	udp_list_init(&handler->w_list);	
	
	handler->remote.sin_family = AF_INET;
  handler->remote.sin_port = htons(remoteport);
	handler->remote.sin_addr.s_addr = inet_addr(remoteip);	
	return (udp_eth_api_handler_t)handler;
}
void udp_eth_delete(udp_eth_api_handler_t* phandler){
	eth_udp_handler_t* handler = (eth_udp_handler_t*)phandler;
	assert(handler);
	udp_mutex_lock(handler->mutex);	
	eth_api_free_list(&handler->r_list);
	eth_api_free_list(&handler->s_list);
	eth_api_free_list(&handler->w_list);
	udp_mp_delete(handler->s_mempool);
	udp_mp_delete(handler->r_mempool);
	udp_mutex_unlock(handler->mutex);	
	udp_mutex_delete(handler->mutex);
	udp_sem_delete(handler->r_sem);
	udp_sem_delete(handler->s_sem);
	udp_free(handler);
}
eth_udp_pack_t* udp_eth_malloc_package(udp_eth_api_handler_t phandler, udp_mp_t mp, int32_t timeout){
	eth_udp_handler_t* 	handler = (eth_udp_handler_t*)phandler;
	eth_udp_pack_t*			pack = NULL;
	assert(handler);	
	//udp_mutex_lock(handler->mutex);
	pack = (eth_udp_pack_t*)udp_mp_malloc(mp, timeout);
	//udp_mutex_unlock(handler->mutex);
	return pack;
}
void udp_eth_free_package(udp_eth_api_handler_t phandler, eth_udp_pack_t* pack){
	eth_udp_handler_t* 	handler = (eth_udp_handler_t*)phandler;
	assert(handler);	
	//udp_mutex_lock(handler->mutex);
	udp_mp_free(pack);
	//udp_mutex_unlock(handler->mutex);
}
int udp_eth_send(udp_eth_api_handler_t phandler, uint8_t* payload, uint32_t payloadLength){
	eth_udp_handler_t* 	handler = (eth_udp_handler_t*)phandler;
	eth_udp_hdr_t 			hdr;
	eth_udp_pack_t*			pack;
	assert(handler);	
	udp_mutex_lock(handler->mutex);
	hdr.version = handler->version;
	hdr.stNo = ++handler->s_no;
	udp_mutex_unlock(handler->mutex);
	
	hdr.sqNo = 1;
	hdr.type = UDP_ETH_SYN;
	hdr.total = payloadLength;
	hdr.position = 0;
	
	while (hdr.position < payloadLength){
		pack = udp_eth_malloc_package(phandler, handler->s_mempool, RT_TICK_PER_SECOND/100);//RT_TICK_PER_SECOND/2
		if (!pack) {
			rt_kprintf("null\n");
			continue;
		}
		udp_list_init(&pack->list);
		pack->header = hdr;
		pack->payloadLength = ((payloadLength - hdr.position) > UDP_ETH_MAXPKTSIZE)?(UDP_ETH_MAXPKTSIZE):((hdr.type = (hdr.type == UDP_ETH_SYN)?(UDP_ETH_SYN):(UDP_ETH_FIN)),payloadLength - hdr.position);
		udp_memcpy(pack->payload, payload + hdr.position, pack->payloadLength);
		hdr.position += pack->payloadLength;
		hdr.sqNo++;
		hdr.type = UDP_ETH_DATA;
		udp_mutex_lock(handler->mutex);
		udp_list_pushback(&handler->s_list, &pack->list);
		udp_mutex_unlock(handler->mutex);		
		udp_sem_set(handler->s_sem);
	}
	return hdr.position;
}
uint32_t udp_eth_isrecv(udp_eth_api_handler_t phandler){
	eth_udp_handler_t* 	handler = (eth_udp_handler_t*)phandler;
	udp_list_t*					node = NULL;
	eth_udp_pack_t*			pack = NULL;
	assert(handler);	
	if (udp_list_isempty(&handler->r_list)) return 0;
	node = udp_list_getfirst(&handler->r_list);
	pack = (eth_udp_pack_t*)udp_list_entry(node, eth_udp_pack_t, list);
	return pack->header.total;
}
int udp_eth_recv(udp_eth_api_handler_t phandler, uint8_t* payload, uint32_t payloadLength){
	eth_udp_handler_t* 	handler = (eth_udp_handler_t*)phandler;
	eth_udp_pack_t*			pack = NULL;
	udp_list_t*					node = NULL;
	uint32_t					payloadLastLength;
	uint32_t					payloadOffset = 0;
	uint32_t					r_no = 0;
	bool							b_No = false;
	
	do{
		udp_sem_get(handler->r_sem);
		node = udp_list_getfirst(&handler->r_list);
		pack = (eth_udp_pack_t*)udp_list_entry(node, eth_udp_pack_t, list);
		payloadLastLength = pack->payloadLength;
		if (!b_No){
			r_no = pack->header.stNo;
			b_No = true;
		}
		else if (pack->header.stNo != r_no){
			udp_sem_set(handler->r_sem);
			break;
		}
		if (payloadLastLength <= (payloadLength - payloadOffset)){
			udp_memcpy(payload + payloadOffset, pack->payload, payloadLastLength);
			payloadOffset+= payloadLastLength;
			udp_mutex_lock(handler->mutex);
			udp_list_remove(node);
			udp_mutex_unlock(handler->mutex);
		}
		else{
			payloadLastLength = payloadLength - payloadOffset;
			udp_memcpy(payload + payloadOffset, pack->payload, payloadLastLength);
			payloadOffset+= payloadLastLength;
			pack->header.position += payloadLastLength;
			pack->payloadLength -= payloadLastLength;
			udp_memcpy(pack->payload, pack->payload+ payloadLastLength, pack->payloadLength);
			udp_sem_set(handler->r_sem);
			break;
		}		
	}while(payloadOffset < payloadLength);
	
	return payloadOffset;	
}

void udp_r_loop(udp_eth_api_handler_t phandler){
	eth_udp_handler_t* 	handler = (eth_udp_handler_t*)phandler;
	eth_udp_pack_t*			pack = NULL;
	udp_list_t*					node = NULL;
	uint32_t						addrsz = sizeof(struct sockaddr_in);
	fd_set fdset;
	struct timeval t;
	int ret = 0;
	t.tv_sec = 0;
	t.tv_usec = 200000;
	while(1){
		FD_ZERO(&fdset);
		FD_SET(handler->sock, &fdset);
    ret = select(FD_SETSIZE, &fdset, NULL, NULL, &t);
		if (ret < 0) break;
		else if (ret > 0){
			pack = udp_eth_malloc_package(phandler, handler->r_mempool, RT_WAITING_FOREVER);
			//if (!pack) { rt_thread_delay(1);continue;}
			udp_list_init(&pack->list);
			//addrsz = sizeof(struct sockaddr_in);
			ret = recvfrom(handler->sock, &pack->header, sizeof(eth_udp_hdr_t) + 4, 0, (struct sockaddr *)&handler->remote, &addrsz);
			if (pack->payloadLength > 0)
				ret = recvfrom(handler->sock, pack->payload, pack->payloadLength, 0, (struct sockaddr *)&handler->remote, &addrsz);
			if (pack->header.type == UDP_ETH_ACK){
				eth_udp_hdr_t header = pack->header;
				udp_eth_free_package(phandler, pack);
				udp_mutex_lock(handler->mutex);
				node = handler->w_list.next;
				while (node != &handler->w_list){
					eth_udp_pack_t*			curr = udp_list_entry(node, eth_udp_pack_t, list);
					if ((header.sqNo == curr->header.sqNo) && (header.stNo == curr->header.stNo)){
						node = curr->list.prev;
						udp_list_remove(&curr->list);
						udp_mp_free(&curr->list);
						break;
					}					
					node = node->next;
				}
				udp_mutex_unlock(handler->mutex);
				continue;
			}
#if UDP_USING_ACK
			else {//ack
				eth_udp_hdr_t header = pack->header;
				eth_udp_pack_t* ack = udp_eth_malloc_package(phandler, handler->s_mempool);
				ack->header = header;
				ack->payloadLength = 0;
				ack->header.type = UDP_ETH_ACK;
				udp_list_pushback(&handler->s_list, &ack->list);
			}		
#endif		
			
			udp_mutex_lock(handler->mutex);
			if (udp_list_isempty(&handler->r_list)){
					udp_list_pushback(&handler->r_list, &pack->list);
			}
			else{
				node = handler->r_list.next;
				while (node != &handler->r_list){
					eth_udp_pack_t*			curr = udp_list_entry(node, eth_udp_pack_t, list);
					if ((curr->header.stNo > pack->header.stNo)
						|| ((curr->header.stNo == pack->header.stNo) && (curr->header.sqNo > pack->header.sqNo))){
						udp_list_pushback(&curr->list, &pack->list);			
						break;
					}						
					node = node->next;
				}	
				if (node == &handler->r_list){
						udp_list_pushback(&handler->r_list, &pack->list);			
				}
				node = &pack->list;
				while (node != &handler->r_list){
					if ((pack->header.stNo == (handler->r_no+1)) && (pack->header.sqNo == (handler->r_sq+1))){
						udp_sem_set(handler->r_sem);
						handler->r_sq++;
						if (pack->header.total <= (pack->header.position + pack->payloadLength)){
							handler->r_sq = 0;
							handler->r_no++;
						}
					}
					else break;
					node = node->next;
				}
			}				
			udp_mutex_unlock(handler->mutex);
		}
#if 0
		else{//没有接收
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==0	
			if (!udp_list_isempty(&handler->w_list)){
				udp_mutex_lock(handler->mutex);
				node = handler->w_list.next;
				while (node != &handler->w_list){
					eth_udp_pack_t*			curr = udp_list_entry(node, eth_udp_pack_t, list);
					node = node->prev;
					udp_list_remove(&curr->list);
					udp_mp_free(curr);		
				}
				udp_mutex_unlock(handler->mutex);
	
			}			
#endif			
		}	
#endif		
	}
}
void udp_s_loop(udp_eth_api_handler_t phandler){
	eth_udp_handler_t* 	handler = (eth_udp_handler_t*)phandler;
	eth_udp_pack_t*			pack = NULL;
	udp_list_t*					node = NULL;
	int 								sbytes = 0;
	int 								sbytes2 = 0;
	while(1){
		udp_sem_get(handler->s_sem);
		node = udp_list_getfirst(&handler->s_list);
		pack = (eth_udp_pack_t*)udp_list_entry(node, eth_udp_pack_t, list);
		udp_mutex_lock(handler->mutex);
		udp_list_remove(node);
		udp_mutex_unlock(handler->mutex);
		sbytes = pack->payloadLength + sizeof(uint32_t) + sizeof(eth_udp_hdr_t);
		sbytes2 = sendto(handler->sock, &pack->header, sbytes, 0, (struct sockaddr*)&handler->remote, sizeof(struct sockaddr_in));
		
		//if (sendto(handler->sock, &pack->header, pack->payloadLength + sizeof(uint32_t) + sizeof(eth_udp_hdr_t), 0, (struct sockaddr*)&handler->remote, sizeof(struct sockaddr_in)) < 0){
		if (sbytes == sbytes2){
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==0	
			udp_mp_free(pack);
#else
			udp_mutex_lock(handler->mutex);
			udp_list_pushback(&handler->w_list, &pack->list);
			udp_mutex_unlock(handler->mutex);
			//rt_kprintf("s\n");
#endif
		}
		else{
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==0	
			udp_mp_free(pack);
#else
			udp_mutex_lock(handler->mutex);
			udp_list_pushback(handler->s_list.next, &pack->list);
			udp_mutex_unlock(handler->mutex);	
			udp_sem_set(handler->s_sem);	
#endif			
			//rt_kprintf("%d\n", errno);
		}
	}
}







