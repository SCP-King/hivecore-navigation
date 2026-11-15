"""Join asynchronous path/review versions without requiring simultaneous arrival."""


class PathReviewPairs:
    def __init__(self):
        self.paths={};self.reviews={}

    def put_path(self,key,path,now):
        self.paths[key]=(path,now)
        self._trim(now)

    def put_review(self,review,now):
        key=review.get('candidate_stamp_ns')
        if isinstance(key,int):self.reviews[key]=(review,now)
        self._trim(now)

    def _trim(self,now):
        for table in (self.paths,self.reviews):
            for key in list(table):
                if now-table[key][1]>.6:del table[key]
            for key in sorted(table)[:-32]:del table[key]

    def newest(self,now):
        self._trim(now)
        for key in sorted(self.paths.keys() & self.reviews.keys(),reverse=True):
            path,_=self.paths[key];review,_=self.reviews[key]
            if review.get('candidate_review_ready') is True:
                return path,review
        return None
