#include "User_Request.h"

namespace SSD_Components
{
	unsigned int User_Request::lastId = 0;

	User_Request::User_Request() : Flash_page_count(0), Sectors_serviced_from_cache(0)
	{
		ID = "" + std::to_string(lastId++);
		ToBeIgnored = false;
	}
}
